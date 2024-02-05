# This limits CPU usage
import os
os.environ["OMP_NUM_THREADS"] = "1" 
os.environ["OPENBLAS_NUM_THREADS"] = "1"
os.environ["MKL_NUM_THREADS"] = "1"
os.environ["VECLIB_MAXIMUM_THREADS"] = "1"
os.environ["NUMEXPR_NUM_THREADS"] = "1"

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.colors import ListedColormap
import sklearn
import pandas as pd
from sklearn.tree import DecisionTreeClassifier
from sklearn.ensemble import AdaBoostClassifier, RandomForestClassifier
from sklearn.inspection import DecisionBoundaryDisplay
from sklearn.model_selection import train_test_split
from sklearn.neighbors import KNeighborsClassifier
from sklearn.pipeline import make_pipeline
from sklearn.preprocessing import StandardScaler
from sklearn.svm import SVC
from sklearn.preprocessing import StandardScaler
from sklearn.decomposition import PCA
from sklearn.utils.class_weight import compute_class_weight

plt.rc('font', size=30)

class ModelComparer:
    def __init__(self, data_files, holdout_test, names, classifiers, get_ave_time=False):
        data_files
        self.n_Pca_Comp = 3
        self.get_ave_time = get_ave_time

        assert (len(names) == len(classifiers))
        self.names = names
        self.classifiers = classifiers

        self.X_train = []
        self.X_test = []
        self.X_train = []
        self.y_test = []
        self.class_names = []
        self.class_weights = []

        self.prepare_data(data_files, holdout_test)

    @staticmethod
    def get_pd(files):
        df_list = []
        for file in files:
            # Read the csv data from a file
            df_list.append(pd.read_csv(file))
        return pd.concat(df_list)
    
    @staticmethod
    def process_pd(df):
         # Add other features
        df['w_h_ratio'] = df['width'] / df['height']
        df['l_h_ratio'] = df['length'] / df['height']
        df['density'] = df['num_points']/df['height']*df['length']*df['width']

        return df

    def prepare_data(self, data_files, test_files):
        '''
        This function takes paths to a list of csv files and prepares the features

        It performs scaling, PCA, and splitting of the train and test
        '''
        df = self.get_pd(data_files)
        ho_test = self.get_pd(test_files)
       
        df = self.process_pd(df)
        ho_test = self.process_pd(ho_test)
        ho_y = ho_test['type']
        ho_test = ho_test.drop(['name', 'type', 'occlusion_level'], axis=1)

        # Separate the features (X) and the target (y)
        X = df.drop(['name', 'type', 'occlusion_level'], axis=1)
        type = df['type']

        # self.get_class_count(type)
        # self.get_feature_stats(df)

        # Encode the labels using label encoding
        le = sklearn.preprocessing.LabelEncoder()
        y = le.fit_transform(type)
        ho_y = le.transform(ho_y)

        # Get class weights, optional for some classifiers
        self.class_weights = compute_class_weight(
            class_weight="balanced", classes=np.unique(y), y=y)
        # Get class names for disply functions
        self.class_names = list(
            le.inverse_transform([0, 1, 2, 3, 4, 5, 6, 7, 8]))

        # Split the data into training and testing sets
        X_train, X_test, y_train, y_test = train_test_split(
            X.values, y, test_size=0.4, random_state=42)
        
        # X_test = ho_test
        # y_test = ho_y

        # Perform data normalization using standard scaling
        scaler = StandardScaler()
        X_train = scaler.fit_transform(X_train)
        X_test = scaler.transform(X_test)

        # Perform PCA analysis to reduce the dimensionality of the features
        pca = PCA(n_components=self.n_Pca_Comp)
        self.X_train = pca.fit_transform(X_train)
        self.X_test = pca.transform(X_test)
        self.y_train = y_train
        self.y_test = y_test

    @staticmethod
    def get_class_count(df):
        series = pd.Series(df)
        # get the frequency of each string
        counts = series.value_counts(sort=True)
        # create a bar plot from the series
        counts.plot(kind='bar', rot=45, title='Class Count')
        plt.show()

    @staticmethod
    def get_feature_stats(df):
        variance = df.var()
        mean = df.mean()

        # Create a new dataframe with the statistics
        stats = pd.DataFrame({
            'variance': variance,
            'mean': mean
        })

        # Plot the statistics as a bar chart
        stats.plot(kind='bar')

    def test_classifier_list(self):
        x_min, x_max = self.X_train[:, 0].min(
        ) - 0.5, self.X_train[:, 0].max() + 0.5
        y_min, y_max = self.X_train[:, 1].min(
        ) - 0.5, self.X_train[:, 1].max() + 0.5

        cm = plt.cm.RdBu
        cm_bright = ListedColormap(["#FF0000", "#0000FF"])

        _, ax = plt.subplots(len(self.classifiers))
        # iterate over classifiers
        for idx, (name, clf) in enumerate(zip(self.names, self.classifiers)):

            clf = make_pipeline(clf)
            clf.fit(self.X_train, self.y_train)
            score = clf.score(self.X_test, self.y_test)

            # Cant plot unless X has dim=(N,2)
            if self.n_Pca_Comp == 2:
                DecisionBoundaryDisplay.from_estimator(
                    clf, self.X_test, cmap=cm, alpha=0.8, ax=ax[idx], eps=0.5
                )

                # Plot the training points
                ax[idx].scatter(
                    self.X_train[:, 0], self.X_train[:,
                                                     1], c=self.y_train, cmap=cm_bright, edgecolors="k"
                )
                # Plot the testing points
                ax[idx].scatter(
                    self.X_test[:, 0],
                    self.X_test[:, 1],
                    c=self.y_test,
                    cmap=cm_bright,
                    edgecolors="k",
                    alpha=0.6,
                )

                ax[idx].set_xlim(x_min, x_max)
                ax[idx].set_ylim(y_min, y_max)
                ax[idx].set_xticks(())
                ax[idx].set_yticks(())
                ax[idx].set_title(name)
                ax[idx].text(
                    x_max - 0.3,
                    y_min + 0.3,
                    ("%.2f" % score).lstrip("0"),
                    size=15,
                    horizontalalignment="right",
                )
            runtime = 0.0
            if self.get_ave_time:
                runtime = self.time_classifier(clf=clf)

            predicted_train = clf.predict(self.X_train)
            results_train = sklearn.metrics.classification_report(self.y_train, predicted_train, target_names=self.class_names, output_dict=True, zero_division=0.0)

            predicted = clf.predict(self.X_test)
            results = sklearn.metrics.classification_report(self.y_test, predicted, target_names=self.class_names, output_dict=True, zero_division=0.0)
            # print(sklearn.metrics.classification_report(self.y_test, predicted, target_names=self.class_names, zero_division=0.0))
            print(f'{name} is {score*100:3.2f} % accurate. Macro ave score is {results["macro avg"]["f1-score"]*100:3.2f}. Runtime is {runtime*1000:.3f} ms. Train acc: {results_train["accuracy"]:.3f} mac {results_train["macro avg"]["f1-score"]*100:3.2f}')

            self.plot_conf_mat(predicted=predicted, name=name)

        plt.show()

    def plot_conf_mat(self, predicted, name, cmd_line=False):
        
        disp = sklearn.metrics.ConfusionMatrixDisplay.from_predictions(
            self.y_test, predicted, display_labels=self.class_names,
            cmap=plt.cm.Blues,
            normalize='true')
        disp.ax_.set_xticklabels(
            disp.ax_.get_xticklabels(), rotation=45, ha='right')
        disp.figure_.suptitle(f'Confusion Matrix {name}')

        if cmd_line:
            print(f"Confusion matrix:\n{disp.confusion_matrix}")

    def time_classifier(self, clf, num_iter=10, num_samples=50):
        from time import time

        assert num_samples < self.X_test.shape[0]

        data = self.X_test[0:num_samples, :]

        start = time()
        for i in range(num_iter):
            clf.predict(data)
        end = time()

        return (end - start)/num_iter
    


def main():

    data_files = [
        '/home/adrian/dev/A9_images_and_points/r02_s01_north_2024-1-26_20-20-17_features.csv',
        '/home/adrian/dev/A9_images_and_points/r02_s04_north_2024-1-24_21-26-12_features.csv',
        '/home/adrian/dev/A9_images_and_points/s02_r01_south_2024-1-24_21-33-51_features.csv',
        '/home/adrian/dev/A9_images_and_points/r02_s02_north2024-1-24_21-30-34_features.csv',
        '/home/adrian/dev/A9_images_and_points/r02_s02_south_2024-1-26_20-17-21_features.csv',
        '/home/adrian/dev/A9_images_and_points/r02_s03_south2023-9-6_11-40-1_features.csv',
        '/home/adrian/dev/A9_images_and_points/r02_s04_south_2024-1-25_9-56-45_features.csv',
        
    ]

    test = [
        '/home/adrian/dev/A9_images_and_points/r02_s01_north_2024-1-26_20-20-17_features.csv',
    ]

    names = [
        # "K-Nearest Neighbors: 5",
        "K-Nearest Neighbors: 7",
        "K-Nearest Neighbors: 9",
        "Linear SVM",
        "RBF SVM",
        "Decision Tree",
        "Random Forest",
        # "Neural Net",
        "AdaBoost",
        # "Naive Bayes",
    ]

    classifiers = [
        # KNeighborsClassifier(5),
        KNeighborsClassifier(7),
        KNeighborsClassifier(13),
        SVC(kernel='linear', gamma=3, C=1, random_state=42, class_weight='balanced'),
        SVC(gamma=3, C=0.6, random_state=42, class_weight='balanced'),
        DecisionTreeClassifier(max_depth=16, random_state=42),
        RandomForestClassifier(
            max_depth=12, n_estimators=12, max_features=2, random_state=43
        ),
        # MLPClassifier(alpha=1, max_iter=1000, random_state=42),
        AdaBoostClassifier(algorithm="SAMME.R", random_state=42),
        # GaussianNB(),
    ]

    md = ModelComparer(data_files=data_files, holdout_test=test, names=names,
                       classifiers=classifiers, get_ave_time=True)

    md.test_classifier_list()


if __name__ == "__main__":
    main()
