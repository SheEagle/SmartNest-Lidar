import numpy as np
import KalmanFilter


def linear_assignment(cost_matrix):
  try:
    import lap
    _, x, y = lap.lapjv(cost_matrix, extend_cost=True)
    return np.array([[y[i],i] for i in x if i >= 0]) #
  except ImportError:
    from scipy.optimize import linear_sum_assignment
    x, y = linear_sum_assignment(cost_matrix)
    return np.array(list(zip(x, y)))


def dis_batch(pos_det, pos_pred):
    """
    :return: 任意两个检测到的与预测的位置距离差
    """
    pos_pred = np.expand_dims(pos_pred, 0)
    pos_det = np.expand_dims(pos_det, 1)
    o = np.sqrt((pos_det[..., 0]-pos_pred[..., 0])**2 + (pos_det[..., 1]-pos_pred[..., 1])**2)
    return o


def associate_detections_to_trackers(detections, trackers, dis_threshold):
    """
    :return: 3 lists of matches, unmatched_detections and unmatched_trackers
    """
    if len(trackers) == 0:
        return np.empty((0, 2), dtype=int), np.arange(len(detections)), np.empty((0, 2), dtype=int)
    if len(detections) == 0:
        return np.empty((0, 2), dtype=int), np.empty((0, 2), dtype=int), np.arange((len(trackers)), dtype=int)

    dis_matrix = dis_batch(detections, trackers)

    if min(dis_matrix.shape) > 0:
        a = (dis_matrix < dis_threshold).astype(np.int32)
        if a.sum(1).max() == 1 and a.sum(0).max() == 1:
            matched_indices = np.stack(np.where(a), axis=1)
        else:
            matched_indices = linear_assignment(dis_matrix)
    else:
        matched_indices = np.empty(shape=(0, 2))

    unmatched_detections = []
    for d, det in enumerate(detections):
        if d not in matched_indices[:, 0]:
            unmatched_detections.append(d)
    unmatched_trackers = []
    for t, trk in enumerate(trackers):
        if t not in matched_indices[:, 1]:
            unmatched_trackers.append(t)

    # filter out matched with low dis
    matches = []
    for m in matched_indices:
        if dis_matrix[m[0], m[1]] > dis_threshold:
            unmatched_detections.append(m[0])
            unmatched_trackers.append(m[1])
        else:
            matches.append(m.reshape(1, 2))
    if len(matches) == 0:
        matches = np.empty((0, 2), dtype=int)
    else:
        matches = np.concatenate(matches, axis=0)

    return matches, np.array(unmatched_detections), np.array(unmatched_trackers)


class KalmanPosTracker(object):
    count = 0
    def __init__(self, pos):
        self.kf = KalmanFilter.KalmanFilter()
        self.kf.state = np.array([[pos[0]], [0], [pos[1]], [0]])
        self.time_since_update = 0
        self.id = KalmanPosTracker.count
        KalmanPosTracker.count += 1
        self.history = []
        self.hits = 0
        self.hit_streak = 0
        self.age = 0

    def update(self, pos):
        self.time_since_update = 0
        self.history = []
        self.hits += 1
        self.hit_streak += 1
        self.kf.correct(pos.reshape(2, 1))

    def predict(self):
        self.kf.predict()
        self.age += 1
        if self.time_since_update > 0:
            self.hit_streak = 0
        self.time_since_update += 1
        self.history.append(np.asarray(self.kf.state).ravel())
        return self.history[-1]

    def get_state(self):
        return np.asarray(self.kf.state).ravel()


class Sort(object):
    def __init__(self, max_age=1, min_hits=3, dis_threshold=800):
        self.max_age = max_age
        self.min_hits = min_hits
        self.dis_threshold = dis_threshold
        self.trackers = []
        self.frame_count = 0

    def update(self, dets=np.empty((0,5))):
        self.frame_count += 1
        trks = np.zeros((len(self.trackers), 2))
        to_del = []
        ret = []
        for t, trk in enumerate(trks):
            trk = self.trackers[t].predict()[[0, 2]]
            trks[t] = trk
            if np.any(np.isnan(trk)):
                to_del.append(t)
        trks = np.ma.compress_rows(np.ma.masked_invalid(trks))  # 删除数组中包含无效值（NaN）的行
        for t in reversed(to_del):
            self.trackers.pop(t)
        matched, unmatched_dets, unmatched_trks = associate_detections_to_trackers(dets, trks, self.dis_threshold)

        # update matched trackers with assigned detections
        for m in matched:
            self.trackers[m[1]].update(dets[m[0], :])

        # create and initialise new trackers for unmatched detections
        for i in unmatched_dets:
            trk = KalmanPosTracker(dets[i, :])
            self.trackers.append(trk)
        i = len(self.trackers)
        for trk in reversed(self.trackers):
            d = trk.get_state()[[0, 2]]
            if trk.time_since_update <= 8 and (trk.hit_streak >= self.min_hits or self.frame_count <= self.min_hits):
                ret.append(np.concatenate((d, [trk.id+1])).reshape(1, -1))
            i -= 1
            # remove dead tracklet
            if trk.time_since_update > self.max_age:
                self.trackers.pop(i)
        if len(ret) > 0:
            return np.concatenate(ret)
        return np.empty((0, 3))

