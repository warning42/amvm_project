#!/usr/bin/env python3
import argparse
from logging import captureWarnings
from tracemalloc import stop
import cv2
import os
import time

import rospy
from std_msgs.msg import Int32

from pycoral.adapters.common import input_size
from pycoral.adapters.detect import get_objects
from pycoral.utils.dataset import read_label_file
from pycoral.utils.edgetpu import make_interpreter
from pycoral.utils.edgetpu import run_inference

def main():
    global cap
    default_model_dir = '/edit_user_dir/'
    default_model = 'mobilenet_ssd_v2_coco_quant_postprocess_edgetpu.tflite'
    default_labels = 'coco_labels_person.txt'
    parser = argparse.ArgumentParser()
    parser.add_argument('--model', help='.tflite model path',
                        default=os.path.join(default_model_dir,default_model))
    parser.add_argument('--labels', help='label file path',
                        default=os.path.join(default_model_dir, default_labels))
    parser.add_argument('--top_k', type=int, default=3,
                        help='number of categories with highest score to display')
    parser.add_argument('--camera_idx', type=int, help='Index of which video source to use. ', default = 0)
    parser.add_argument('--threshold', type=float, default=0.1,
                        help='classifier score threshold')
    args = parser.parse_args()

    print('Loading {} with {} labels.'.format(args.model, args.labels))
    interpreter = make_interpreter(args.model)
    interpreter.allocate_tensors()
    labels = read_label_file(args.labels)
    inference_size = input_size(interpreter)

    cap = cv2.VideoCapture(args.camera_idx)
    last_time = time.monotonic()
    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break
        frame = cv2.flip(frame,1)
        cv2_im = frame

        cv2_im_rgb = cv2.cvtColor(cv2_im, cv2.COLOR_BGR2RGB)
        cv2_im_rgb = cv2.resize(cv2_im_rgb, inference_size)
        start_time = time.monotonic()
        run_inference(interpreter, cv2_im_rgb.tobytes())
        objs = get_objects(interpreter, args.threshold)[:args.top_k]
        stop_time = time.monotonic()
        inference_ms = (stop_time - start_time)*1000.0
        fps_ms = 1.0 / (stop_time - last_time)
        annotate_text = 'Inference: {:5.2f}ms FPS: {:3.1f}'.format(inference_ms, fps_ms)
        last_time = stop_time
        cv2_im = append_objs_to_img(cv2_im, inference_size, objs, labels)

        print(annotate_text)
        try:
            print(position)
        except:
            pass
        cv2.imshow('frame', cv2_im)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        pub = rospy.Publisher('camera_chatter', Int32, queue_size=1)
        rospy.init_node('talker', anonymous=False)
        pub.publish(position)
        if rospy.is_shutdown():
            break
    cap.release()
    cv2.destroyAllWindows()

def append_objs_to_img(cv2_im, inference_size, objs, labels):
    global position
    height, width, channels = cv2_im.shape
    width_split = width // 5
    scale_x, scale_y = width / inference_size[0], height / inference_size[1]  # height = 720, width = 1080, inference_size = (300, 300)
    cv2_im = cv2.line(cv2_im, (width_split*1 - 20, 0), (width_split*1 - 20, height), (255,0,0), 5)
    cv2_im = cv2.line(cv2_im, (width_split*2 - 5, 0), (width_split*2 - 5, height), (255,0,0), 5)
    cv2_im = cv2.line(cv2_im, (width_split*3 + 5, 0), (width_split*3 + 5, height), (255,0,0), 5)
    cv2_im = cv2.line(cv2_im, (width_split*4 + 20, 0), (width_split*4 + 20, height), (255,0,0), 5)
    stop_num = 0
    for obj in objs:
        bbox = obj.bbox.scale(scale_x, scale_y)
        x0, y0 = int(bbox.xmin), int(bbox.ymin)
        x1, y1 = int(bbox.xmax), int(bbox.ymax)
        x2, y2 = (int(bbox.xmin) + int(bbox.xmax))//2, (int(bbox.ymin) + int(bbox.ymax))//2
        size_of_box = (x1-x0) * (y1-y0)
        percent = int(100 * obj.score)
        label = '{}% {}'.format(percent, labels.get(obj.id, obj.id))
        if labels.get(obj.id, obj.id) == 'person':
            cv2_im = cv2.rectangle(cv2_im, (x0, y0), (x1, y1), (0, 255, 0), 2)
            cv2_im = cv2.putText(cv2_im, label + f'{size_of_box}', (x0, y0+30),
                                cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 0, 0), 2)
            cv2_im = cv2.circle(cv2_im, (x2, y2), 10, (0, 0, 255), -1)
            if x2 <= width_split*1 - 20:
                stop_num = 1
                position = -2
            elif x2 <= width_split*2 - 5:
                stop_num = 1
                position = -1
            elif x2 <= width_split*3 + 5:
                stop_num = 1
                position = 0
            elif x2 <= width_split*4 + 20:
                stop_num = 1
                position = 1 
            else:
                stop_num = 1
                position = 2
            if stop_num == 1 and size_of_box >= 400000:
                position = 10000
                rospy.is_shutdown()
                cap.release()
                cv2.destroyAllWindows()
        else:
            pass
    if stop_num == 0:
        position = 10
    return cv2_im

if __name__ == '__main__':
    main()
