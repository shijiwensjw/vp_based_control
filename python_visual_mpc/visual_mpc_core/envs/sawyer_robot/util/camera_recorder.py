import numpy as np
import rospy
from threading import Lock, Semaphore
from cv_bridge import CvBridge
import cv2
from sensor_msgs.msg import Image as Image_msg
import copy


class LatestObservation(object):
    def __init__(self, create_tracker=False, save_buffer=False):
        self.img_cv2 = None
        self.tstamp_img = None
        self.img_msg = None
        self.mutex = Lock()
        if save_buffer:
            self.reset_saver()
        if create_tracker:
            self.reset_tracker()

    def reset_tracker(self):
        self.cv2_tracker = cv2.TrackerMIL_create()
        self.bbox = None
        self.track_itr = 0

    def reset_saver(self):
        self.save_itr = 0


class CameraRecorder:
    TRACK_SKIP = 2        # the camera publisher works at 60 FPS but camera itself only goes at 30

    def __init__(self, topic_name, opencv_tracking=False, save_videos=False):
        self._tracking_enabled, self._save_vides = opencv_tracking, save_videos

        self._latest_image = LatestObservation(self._tracking_enabled, self._save_vides)

        self._is_tracking = False
        if self._tracking_enabled:
            self.box_height = 80

        self.bridge = CvBridge()
        self._first_status, self._status_sem = False, Semaphore(value=0)
        self._cam_height, self._cam_width = None, None
        if self._save_vides:
            self._buffers = []
            self._saving = False

        rospy.Subscriber(topic_name, Image_msg, self.store_latest_im)
        print('downing sema on topic: {}'.format(topic_name))
        self._status_sem.acquire()
        print("Cameras subscribed: stream is {}x{}".format(self._cam_width, self._cam_height))

    def _cam_start_tracking(self, lt_ob, point):
        lt_ob.reset_tracker()
        lt_ob.bbox = np.array([int(point[1] - self.box_height / 2.),
                               int(point[0] - self.box_height / 2.),
                               self.box_height, self.box_height]).astype(np.int64)

        lt_ob.cv2_tracker.init(lt_ob.img_cv2, tuple(lt_ob.bbox))
        lt_ob.track_itr = 0

    def start_tracking(self, start_points):
        assert self._tracking_enabled
        n_desig, xy_dim = start_points.shape
        if n_desig != 1:
            raise NotImplementedError("opencv_tracking requires 1 designated pixel")
        if xy_dim != 2:
            raise ValueError("Requires XY pixel location")

        self._latest_image.mutex.acquire()
        self._cam_start_tracking(self._latest_image, start_points[0])
        self._is_tracking = True
        self._latest_image.mutex.release()
        rospy.sleep(2)   # sleep a bit for first few messages to initialize tracker

        print("TRACKING INITIALIZED")

    def end_tracking(self):
        self._latest_image.mutex.acquire()
        self._is_tracking = False
        self._latest_image.reset_tracker()
        self._latest_image.mutex.release()

    def _bbox2point(self, bbox):
        point = np.array([int(bbox[1]), int(bbox[0])]) \
                  + np.array([self.box_height / 2, self.box_height / 2])
        return point.astype(np.int32)

    def get_track(self):
        assert self._tracking_enabled, "OPENCV TRACKING IS NOT ENABLED"
        assert self._is_tracking, "RECORDER IS NOT TRACKING"

        points = np.zeros((1, 2), dtype=np.int64)
        self._latest_image.mutex.acquire()
        points[0] = self._bbox2point(self._latest_image.bbox)
        self._latest_image.mutex.release()

        return points.astype(np.int64)

    def get_image(self):
        self._latest_image.mutex.acquire()
        time_stamp, img_cv2 = self._latest_image.tstamp_img, self._latest_image.img_cv2
        self._latest_image.mutex.release()

        return time_stamp, img_cv2

    def start_recording(self, reset_buffer=False):
        assert self._save_vides, "Video saving not enabled!"

        self._latest_image.mutex.acquire()
        if reset_buffer:
            self.reset_recording()
        self._saving = True
        self._latest_image.mutex.release()

    def stop_recording(self):
        assert self._save_vides, "Video saving not enabled!"
        self._latest_image.mutex.acquire()
        self._saving = False
        self._latest_image.mutex.release()

    def reset_recording(self):
        assert self._save_vides, "Video saving not enabled!"
        assert not self._saving, "Can't reset while saving (run stop_recording first)"

        old_buffers = self._buffers
        self._buffers = []
        self._latest_image.reset_saver()
        return old_buffers

    def _proc_image(self, latest_obsv, data):
        latest_obsv.img_msg = data
        latest_obsv.tstamp_img = rospy.get_time()

        cv_image = self.bridge.imgmsg_to_cv2(data, "bgra8")[:, :, :3]
        latest_obsv.img_cv2 = copy.deepcopy(cv_image)

        if self._tracking_enabled and self._is_tracking:
            if latest_obsv.track_itr % self.TRACK_SKIP == 0:
                _, bbox = latest_obsv.cv2_tracker.update(latest_obsv.img_cv2)
                latest_obsv.bbox = np.array(bbox).astype(np.int32).reshape(-1)
            latest_obsv.track_itr += 1

    def store_latest_im(self, data):
        self._latest_image.mutex.acquire()
        self._proc_image(self._latest_image, data)

        if not self._first_status:
            self._cam_height, self._cam_width = self._latest_image.img_cv2.shape[:2]
            self._first_status = True
            self._status_sem.release()

        if self._save_vides and self._saving:
            if self._latest_image.save_itr % self.TRACK_SKIP == 0:
                self._buffers.append(copy.deepcopy(self._latest_image.img_cv2)[:, :, ::-1])
            self._latest_image.save_itr += 1
        self._latest_image.mutex.release()

    @property
    def img_width(self):
        return self._cam_width

    @property
    def img_height(self):
        return self._cam_height
