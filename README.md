# vision_tune

## 노드 구성:
 ### -ui node(15hz ui_tick함수에서 토픽 동작, mainwindow는 qt제어)
   #### : hsv튜닝 용도
 ### -process node(30hz process_tick함수에서 토픽과 이미지 처리 동작)
   #### : raw이미지와 hsv값 받아 이미지 처리 후 결과 발행
 ### -camera node(15hz)
   #### : 카메라 제어, raw이미지 발행
 ### 모든 노드에서 토픽 콜백 함수는 받은 값을 버퍼에 저장만, 처리는 hz로 제어되는 tick함수에서
 
