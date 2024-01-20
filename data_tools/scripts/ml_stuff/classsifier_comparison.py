
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.colors import ListedColormap
import sklearn
import pandas as pd
from sklearn.ensemble import AdaBoostClassifier, RandomForestClassifier
from sklearn.inspection import DecisionBoundaryDisplay
from sklearn.model_selection import train_test_split
from sklearn.naive_bayes import GaussianNB
from sklearn.neighbors import KNeighborsClassifier
from sklearn.neural_network import MLPClassifier
from sklearn.pipeline import make_pipeline
from sklearn.preprocessing import StandardScaler
from sklearn.svm import SVC
from sklearn.tree import DecisionTreeClassifier
from sklearn.preprocessing import StandardScaler
from sklearn.decomposition import PCA
from sklearn.utils.class_weight import compute_class_weight


class ModelComparer:
    def __init__(self, data_path, names, classifiers):
        self.path = data_path
        self.n_Pca_Comp = 3

        assert (len(names) == len(classifiers))
        self.names = names
        self.classifiers = classifiers

        self.X_train = []
        self.X_test = []
        self.X_train = []
        self.y_test = []
        self.class_names = []
        self.class_weights = []

        self.prepare_data(self.path)

    def prepare_data(self, path):
        '''
        This function takes a path to a csv file and prepares the features

        It performs scaling, PCA, and splitting of the train and test
        '''
        # Read the csv data from a file
        df = pd.read_csv(path)

        # Add other features
        df['w_h_ratio'] = df['width'] / df['height']
        df['l_h_ratio'] = df['length'] / df['height']
        df['density'] = df['num_points']/df['height']*df['length']*df['width']

        # Separate the features (X) and the target (y)
        X = df.drop(['name', 'type', 'occlusion_level'], axis=1)
        type = df['type']

        # Encode the labels using label encoding
        le = sklearn.preprocessing.LabelEncoder()
        y = le.fit_transform(type)

        self.class_weights = compute_class_weight(
            class_weight="balanced", classes=np.unique(y), y=y)
        self.class_names = list(
            le.inverse_transform([0, 1, 2, 3, 4, 5, 6, 7, 8]))

        # Split the data into training and testing sets (80/20 ratio)
        X_train, X_test, y_train, y_test = train_test_split(
            X.values, y, test_size=0.2, random_state=42)  # remove values if it causes a problem

        # Perform data normalization using standard scaling
        scaler = StandardScaler()
        X_train = scaler.fit_transform(X_train)
        X_test = scaler.transform(X_test)

        # Perform PCA analysis to reduce the dimensionality of the features
        pca = PCA(tol=.98)
        self.X_train = pca.fit_transform(X_train)
        self.X_test = pca.transform(X_test)
        self.y_train = y_train
        self.y_test = y_test

    def test_classifier(self):
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

            self.plot_conf_mat(clf=clf, name=name)

            print(f'{name} is "{score*100:3.1f} %" accurate.')

        plt.show()

    def plot_conf_mat(self, clf, name, cmd_line=False):
        predicted = clf.predict(self.X_test)
        disp = sklearn.metrics.ConfusionMatrixDisplay.from_predictions(
            self.y_test, predicted, display_labels=self.class_names,
            cmap=plt.cm.Blues,
            normalize='true')
        disp.ax_.set_xticklabels(
            disp.ax_.get_xticklabels(), rotation=45, ha='right')
        disp.figure_.suptitle(f'Confusion Matrix {name}')

        if cmd_line:
            print(f"Confusion matrix:\n{disp.confusion_matrix}")


def main():

    names = [
        "K-Nearest Neighbors: 3",
        "K-Nearest Neighbors: 5",
        "K-Nearest Neighbors: 7",
        "K-Nearest Neighbors: 9",
        "RBF SVM weighted",
        "RBF SVM",
        # "Decision Tree",
        # "Random Forest",
        # "Neural Net",
        # "AdaBoost",
        # "Naive Bayes",
    ]

    classifiers = [
        KNeighborsClassifier(3),
        KNeighborsClassifier(5),
        KNeighborsClassifier(7),
        KNeighborsClassifier(9),
        SVC(gamma=2, C=1, random_state=42, class_weight='balanced'),
        SVC(gamma=2, C=1, random_state=42),
        # DecisionTreeClassifier(max_depth=5, random_state=42),
        # RandomForestClassifier(
        #     max_depth=5, n_estimators=10, max_features=1, random_state=42
        # ),
        # MLPClassifier(alpha=1, max_iter=1000, random_state=42),
        # AdaBoostClassifier(algorithm="SAMME", random_state=42),
        # GaussianNB(),
    ]

    path = "/home/adrian/dev/A9_images_and_points/a9_dataset_r02_s03/2023-9-6_11-40-1_features.csv"
    md = ModelComparer(data_path=path, names=names, classifiers=classifiers)

    md.test_classifier()


if __name__ == "__main__":
    main()
