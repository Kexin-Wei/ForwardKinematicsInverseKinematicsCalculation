import numpy as np


class KalmanFilter:
    def __init__(
        self,
        x0: np.ndarray,
        P0: np.ndarray,
        F: np.ndarray,
        Q: np.ndarray,
        H: np.ndarray,
        R: np.ndarray,
    ):
        assert x0.shape[0] > x0.shape[1], "x0 must be a column vector"
        self.x: np.ndarray = x0
        assert P0.shape[0] == P0.shape[1], "P0 must be a square matrix"
        self.P: np.ndarray = P0
        assert F.shape[0] == F.shape[1], "F must be a square matrix"
        self.F: np.ndarray = F
        assert Q.shape[0] == Q.shape[1], "Q must be a square matrix"
        self.Q: np.ndarray = Q
        self.H: np.ndarray = H
        assert R.shape[0] == R.shape[1], "R must be a square matrix"
        self.R: np.ndarray = R

    def predict(self):
        self.x = self.F @ self.x
        self.P = self.F @ self.P @ self.F.T + self.Q

    def update(self, z: np.ndarray):
        dz = z - self.H @ self.x
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)
        assert self.x.shape == (K @ dz).shape, "self.x 's shape must be preserved"
        self.x = self.x + K @ dz
        self.P = self.P - K @ self.H @ self.P


if __name__ == "__main__":
    from pathlib import Path
    from .folder import FolderMg
    import pandas as pd
    import matplotlib.pyplot as plt

    dataPath = Path("data").joinpath("motion_tracking")
    dataMg = FolderMg(dataPath)
    # dataMg.ls()
    file = dataMg.files[2]
    print("Read file", file.name)

    df = pd.read_csv(file, header=None)
    df.head()

    pos_collection = []
    for index, row in df.iterrows():
        transform_matrix = row[1:17]
        pos = np.array([transform_matrix[4], transform_matrix[8], transform_matrix[12]])
        pos_collection.append(pos)
    pos_collection = np.array(pos_collection)
    print(pos_collection.shape)

    timestamp = df[[0]].to_numpy().squeeze()
    print(timestamp.shape)

    T = len(timestamp)
    T = 50
    trajectory = pos_collection[:T]
    t = timestamp[:T]
    x = trajectory[:, 0]
    y = trajectory[:, 1]
    N = len(y)
    yob = y + np.random.randn(N) * 5
    dt = np.diff(t).mean()

    X = np.zeros((3, 1))
    F = np.array([[1, dt, 0.5 * dt**2], [0, 1, dt], [0, 0, 1]])
    H = np.array([[1, 0, 0]])
    P = np.eye(3)
    Q = np.eye(3) * 0.5
    R = np.eye(1) * 0.5
    kf = KalmanFilter(X, P, F, Q, H, R)

    x_est = []
    for i in range(N):
        kf.predict()
        kf.update(yob[i])
        x_est.append(kf.x[0])

    x_est = np.array(x_est).squeeze()
    fig = plt.figure(figsize=(10, 4))
    ax = fig.add_subplot(211)
    ax.plot(t, y, "k*-", label="original")
    ax.plot(t, yob, "bo", label="observation")
    ax.plot(t, x_est, "r*-", label="estimate")
    ax.legend()
    ax.set_xlabel("time [s]")
    ax.set_ylabel("position [m]")
    ax = fig.add_subplot(212)
    ax.plot(t, x_est - y, "g*-", label="error")
    ax.legend()
    ax.set_xlabel("time [s]")
    ax.set_ylabel("error [m]")
    plt.show()
