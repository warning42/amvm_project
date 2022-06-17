# amvm_project

> 종합 설계: 자율주행 자판기 로봇 프로젝트

## 라이브러리 및 환경
ubuntu 20.04 noetic

python3

ROS: 로봇 운영체제

pycoral: 구글 코랄에서 제작한 딥러닝 전용 라이브러리 (라이브러리 출처: coral.ai)

opencv: 이미지 프레임 캡쳐를 위한 라이브러리

arduino program: opencr board로 ino파일 업로드를 위함

## model
구글 코랄에서 제공하는 edge tpu 전용 딥러닝 모델인 mobilenet v2 for edgetpu를 이용

## label
현재는 사람만 인식하게 코드(detect_pub.py, coco_labels_person 참고)를 작성하였으나, 인식 객체를 변경하고 싶다면

coco labels original을 참고하여 변경.

## scripts
detect_pub.py: 객체인식부터 객체의 포지션의 publish까지 담당

motor_control.py: 객체인식으로부터 얻은 토픽과 더불어 초음파 거리 측정 토픽, 서보모터 토픽를 총 이용하여 모터 제어를 위한 토픽을 발행

## opencr
amvm_opencr.ino: 받은 토픽을 기반으로 각 모터로 시리얼로 제어 신호를 전달


