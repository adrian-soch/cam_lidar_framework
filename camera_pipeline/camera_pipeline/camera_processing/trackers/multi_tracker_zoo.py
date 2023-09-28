from trackers.strong_sort.utils.parser import get_config
from trackers.strong_sort.strong_sort import StrongSORT
from trackers.ocsort.ocsort import OCSort
# from trackers.bytetrack.byte_tracker import BYTETracker


def create_tracker(tracker_type, appearance_descriptor_weights, device, half, config_path_root=None):
    if tracker_type == 'strongsort':
        # initialize StrongSORT
        cfg = get_config()
        if config_path_root is not None:
            cfg_path = config_path_root + '/trackers/strong_sort/configs/strong_sort.yaml'
        else:
            cfg_path = '/trackers/strong_sort/configs/strong_sort.yaml'
        cfg.merge_from_file(cfg_path)
        
        strongsort = StrongSORT(
            appearance_descriptor_weights,
            device,
            half,
            max_dist=cfg.STRONGSORT.MAX_DIST,
            max_iou_distance=cfg.STRONGSORT.MAX_IOU_DISTANCE,
            max_age=cfg.STRONGSORT.MAX_AGE,
            n_init=cfg.STRONGSORT.N_INIT,
            nn_budget=cfg.STRONGSORT.NN_BUDGET,
            mc_lambda=cfg.STRONGSORT.MC_LAMBDA,
            ema_alpha=cfg.STRONGSORT.EMA_ALPHA,

        )
        return strongsort
    elif tracker_type == 'ocsort':
        ocsort = OCSort(
            det_thresh=0.45,
            iou_threshold=0.2,
            use_byte=False 
        )
        return ocsort
    # elif tracker_type == 'bytetrack':
    #     bytetracker = BYTETracker(
    #         track_thresh=0.6,
    #         track_buffer=30,
    #         match_thresh=0.8,
    #         frame_rate=30
    #     )
    #     return bytetracker
    else:
        print('No such tracker')
        exit()