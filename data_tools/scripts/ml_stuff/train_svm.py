import pandas as pd
import sklearn
from sklearn.model_selection import train_test_split
from sklearn.preprocessing import StandardScaler
from sklearn.decomposition import PCA
from sklearn.svm import SVC
from sklearn.inspection import DecisionBoundaryDisplay
import matplotlib.pyplot as plt
from joblib import dump

MODEL_PATH = "/home/adrian/dev/A9_images_and_points/a9_dataset_r02_s03"

# Read the csv data from a file
df = pd.read_csv(
    "/home/adrian/dev/A9_images_and_points/a9_dataset_r02_s03/2023-9-6_11-40-1_features.csv")

# Add other features
df['w_h_ratio'] = df['width'] / df['height']
df['l_h_ratio'] = df['length'] / df['height']

# Separate the features (X) and the target (y)
X = df.drop(['name', 'type', 'occlusion_level'], axis=1)
y = df['type']

# Encode the labels using label encoding
le = sklearn.preprocessing.LabelEncoder()
y = le.fit_transform(y)

# Split the data into training and testing sets (80/20 ratio)
X_train, X_test, y_train, y_test = train_test_split(
    X.values, y, test_size=0.2, random_state=42)  # remove values if it causes a problem

# Perform data normalization using standard scaling
scaler = StandardScaler()
X_train = scaler.fit_transform(X_train)
X_test = scaler.transform(X_test)

# Perform PCA analysis to reduce the dimensionality of the features
pca = PCA(n_components=2)
X_train = pca.fit_transform(X_train)
X_test = pca.transform(X_test)

# Perform SVM training using a linear kernel and default parameters
svm = SVC(kernel="rbf", C=0.7, cache_size=500, random_state=42)
svm.fit(X_train, y_train)

# Save the scaling, PCA and SVM parameters to a yaml file for later use
dump(scaler, f'{MODEL_PATH}/scaler.joblib')
dump(pca, f'{MODEL_PATH}/pca.joblib')
dump(svm, f'{MODEL_PATH}/svm.joblib')

# # Plot the reduced data with different colors for each class
# plt.figure(figsize=(8, 6))
# plt.scatter(X_train[:, 0], X_train[:, 1], c=y_train, cmap="rainbow")
# plt.xlabel("PC1")
# plt.ylabel("PC2")
# plt.title("PCA of training data")
# plt.show()

# Print the accuracy score of the SVM classifier on the testing set
print(f"Accuracy score: {svm.score(X_test, y_test)}")

# Predict the value of the digit on the test subset
predicted = svm.predict(X_test)

ax = plt.gca()
DecisionBoundaryDisplay.from_estimator(
    svm, X_test, cmap='viridis', alpha=0.8, ax=ax, eps=0.5
)
# Plot the training points
ax.scatter(
    X_train[:, 0], X_train[:, 1], c=y_train, cmap='viridis', edgecolors="k"
)
# Plot the testing points
ax.scatter(
    X_test[:, 0],
    X_test[:, 1],
    c=y_test,
    cmap='viridis',
    edgecolors="k",
    alpha=0.6,
)

plt.show()

class_names = list(le.inverse_transform([0, 1, 2, 3, 4, 5, 6, 7, 8]))
print(class_names)

# disp = sklearn.metrics.ConfusionMatrixDisplay.from_predictions(
#     y_test, predicted, display_labels=class_names,
#         cmap=plt.cm.Blues,
#         normalize='true')
# disp.figure_.suptitle("Confusion Matrix")
# print(f"Confusion matrix:\n{disp.confusion_matrix}")
# plt.show()

# Outlier test
# X_new = np.array((8, 8)).reshape(1, -1)
# y_new = svm.decision_function(X_new)
# print(y_new)
