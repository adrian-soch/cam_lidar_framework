"""
Modified by: Adrian Sochaniwsky
Based on:

    SORT: A Simple, Online and Realtime Tracker
    Copyright (C) 2016-2020 Alex Bewley alex@bewley.ai

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
"""

from filterpy.kalman import KalmanFilter
from lap import lapjv
import numpy as np
from scipy.spatial.distance import cdist

np.random.seed(0)

def linear_assignment(cost_matrix):
    _, x, y = lapjv(cost_matrix, extend_cost=True)
    return np.array([[y[i], i] for i in x if i >= 0])

def centroid_distance_batch(dets, trackers):
    d_pos = dets[:,:3]
    t_pos = trackers[:,:3]

    return cdist(d_pos, t_pos, metric='euclidean')

class KalmanBoxTracker(object):
    """
    This class represents the internal state of individual tracked objects observed as bbox.
    """
    count = 0
    def __init__(self, z, sensor_type, dt=0.1, dim_z=1):
        """
        Initialises a tracker using initial bounding box.
        """

        self.P_X, self.P_Y, self.P_Z, self.YAW, self.LENGTH, self.WIDTH, self.HEIGHT, self.V_X, self.V_Y, self.dW_YAW = range(
                10)
        
        # Constant velocity model for x,y,yaw. Constant position model for the rest.
        self.kf = KalmanFilter(dim_x=10, dim_z=dim_z)
        self.kf.F = np.array([[1,0,0,0,0,0,0,dt,0,0], # P_X
                              [0,1,0,0,0,0,0,0,dt,0], # P_Y
                              [0,0,1,0,0,0,0,0,0,0],  # P_Z
                              [0,0,0,1,0,0,0,0,0,dt], # YAW
                              [0,0,0,0,1,0,0,0,0,0],  # L
                              [0,0,0,0,0,1,0,0,0,0],  # W
                              [0,0,0,0,0,0,1,0,0,0],  # H
                              [0,0,0,0,0,0,0,1,0,0],  # V_X
                              [0,0,0,0,0,0,0,0,1,0],  # V_Y
                              [0,0,0,0,0,0,0,0,0,1]]) # D_YAW
        
        self.kf.H = np.array([[1,0,0,0,0,0,0,0,0,0],
                              [0,1,0,0,0,0,0,0,0,0],
                              [0,0,1,0,0,0,0,0,0,0],
                              [0,0,0,1,0,0,0,0,0,0],
                              [0,0,0,0,1,0,0,0,0,0],
                              [0,0,0,0,0,1,0,0,0,0],
                              [0,0,0,0,0,0,1,0,0,0]])

        self.kf.R[2:,2:] *= 10.
        self.kf.P[7:,7:] *= 1000. #give high uncertainty to the unobservable initial velocities
        self.kf.P *= 10.
        self.kf.Q[-1,-1] *= 0.01
        self.kf.Q[7:,7:] *= 0.01

        # Init state with measurement
        self.kf.x[:dim_z] = np.expand_dims(z[:-1], axis=1)
        self.time_since_update = 0
        self.id = KalmanBoxTracker.count
        KalmanBoxTracker.count += 1
        self.history = []
        self.hits = 0
        self.hit_streak = 0
        self.age = 0
        self.sensor_type = sensor_type
        self.dt = dt
        self.dim_z = dim_z

    def update(self, z, sensor_type):
        """
        Updates the state vector with observed bbox.
        """
        self.time_since_update = 0
        self.history = []
        self.hits += 1
        self.hit_streak += 1
        self.kf.update(z[:-1])
        self.sensor_type = sensor_type

    def predict(self):
        """
        Advances the state vector and returns the predicted bounding box estimate.
        """
        self.kf.predict()
        self.age += 1
        if(self.time_since_update>0):
            self.hit_streak = 0
        self.time_since_update += 1
        self.history.append(self.kf.x)
        return self.history[-1]

    def get_state(self):
        """
        Returns the current state estimate.
        """
        return self.kf.x[:]


def associate_detections_to_trackers(detections,trackers,dist_threshold = 0.3):
    """
    Assigns detections to tracked object (both represented as bounding boxes)

    Returns 3 lists of matches, unmatched_detections and unmatched_trackers
    """
    if(len(trackers)==0):
        return np.empty((0,2),dtype=int), np.arange(len(detections)), np.empty((0,5),dtype=int)

    cost_matrix = centroid_distance_batch(detections, trackers)

    if min(cost_matrix.shape) > 0:
        a = (cost_matrix > dist_threshold).astype(np.int32)
        if a.sum(1).max() == 1 and a.sum(0).max() == 1:
            matched_indices = np.stack(np.where(a), axis=1)
        else:
            matched_indices = linear_assignment(-cost_matrix)
    else:
        matched_indices = np.empty(shape=(0,2))

    unmatched_detections = []
    for d, det in enumerate(detections):
        if(d not in matched_indices[:,0]):
            unmatched_detections.append(d)
    unmatched_trackers = []
    for t, trk in enumerate(trackers):
        if(t not in matched_indices[:,1]):
            unmatched_trackers.append(t)

    #filter out matched with low IOU
    matches = []
    for m in matched_indices:
        if(cost_matrix[m[0], m[1]]<dist_threshold):
            unmatched_detections.append(m[0])
            unmatched_trackers.append(m[1])
        else:
            matches.append(m.reshape(1,2))
    if(len(matches)==0):
        matches = np.empty((0,2),dtype=int)
    else:
        matches = np.concatenate(matches,axis=0)

    return matches, np.array(unmatched_detections), np.array(unmatched_trackers)


class Sort(object):
    def __init__(self, max_age=3, min_hits=3, dist_threshold=3.0, dt=0.1):
        """
        Sets key parameters for SORT
        """
        self.max_age = max_age
        self.min_hits = min_hits
        self.dist_threshold = dist_threshold
        self.trackers = []
        self.frame_count = 0
        self.dim_z = 7
        self.dt = dt

    def update(self, dets=np.empty((0, 7))):
        """
        Params:
        dets - a numpy array of detections in the format [[x,y,z,yaw,h,l,w],[x,y,z,yaw,h,l,w],...]
        Requires: this method must be called once for each frame even with empty detections (use np.empty((0, 7)) for frames without detections).
        Returns the a similar array, with appended track iD and sensor type.

        NOTE: The number of objects returned may differ from the number of detections provided.
        """
        self.frame_count += 1
        # get predicted locations from existing trackers.
        trks = np.zeros((len(self.trackers), self.dim_z))
        to_del = []
        ret = []
        for t, trk in enumerate(trks):
            pos = self.trackers[t].predict()[0]
            trk[:] = pos
            if np.any(np.isnan(pos)):
                to_del.append(t)
        trks = np.ma.compress_rows(np.ma.masked_invalid(trks))
        for t in reversed(to_del):
            self.trackers.pop(t)
        matched, unmatched_dets, unmatched_trks = associate_detections_to_trackers(dets,trks, self.dist_threshold)

        # update matched trackers with assigned detections
        for m in matched:
            self.trackers[m[1]].update(dets[m[0], :], dets[m[0], -1])

        # create and initialise new trackers for unmatched detections
        for i in unmatched_dets:
            trk = KalmanBoxTracker(dets[i,:], dt=self.dt, dim_z=self.dim_z, sensor_type=dets[i,-1])
            self.trackers.append(trk)
        i = len(self.trackers)
        for trk in reversed(self.trackers):
            d = np.array(trk.get_state()).T.reshape(-1)
            if (trk.time_since_update < self.max_age) and (trk.hit_streak >= self.min_hits or self.frame_count <= self.min_hits):
                ret.append(np.concatenate((d,[trk.id+1, trk.sensor_type])).reshape(1,-1)) # +1 as MOT benchmark requires positive
            i -= 1
            # remove dead tracklet
            if(trk.time_since_update > self.max_age):
                self.trackers.pop(i)
        if(len(ret)>0):
            return np.concatenate(ret)
        return np.empty((0,self.dim_z))
