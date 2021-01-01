# Smart Walker

광운대학교 로봇학부 학술소모임 **'BARAM'** 20년도 전반기 **'Smart Walker'** Project에 대한 소스코드입니다.  

## 개발 환경
|OS|사용 언어|사용 IDE|
|:---:|:---:|:---:|
| Window 10 | C | Cube MX, IAR |

## 프로젝트 개발 동기

-  노인의 신체 능력은 좋지 않기 때문에 여러 사고가 일어날 수 있는데, 특히 시각적으로 신호등을 잘 보지 못한다는 것과 오르막길, 내리막길에서 보행에 어려움을 느끼는 것으로부터 생기는 안전사고가  다수이다. 특히 신호등에 나타나는 숫자를 제대로 보지 못하고 신호등을 건너다 신호등 불이 바뀌 는 위험한 상황도 대다수가 존재한다.

-  따라서, 이 스마트 보행기는 사용자의 오르막길, 내리막길에서의 보행에 보조를 해주어 안전사고를 예방하고, 신호등 및 남은시간을 계산하여 횡단보 도를 안전하게 건널 수 있도록 할 것이다.
-  **본 프로젝트는 하드웨어 부분에 대한 코드만 나와있습니다** 
## 프로젝트 개요
1.   보행자의 진행 방향을 Encoder를 통해 조정
  >* 손잡이 부분의 Encoder 센서를 부착하여, 손잡이 부분을 돌리는 각	도, 방향에 따라 보행 방향을 측정한다. 
2.   보행자와 보행기간의 거리를 PSD센서를 통해 측정
  >* PSD센서를 활용하여 사용자와 보행기 사이의 거리를 측정하여 사용	자의 속도에 맞게 보행기의 속도 및 정지를 구현하는데 활용한다.
3.   방향 및 거리를 PD제어를 통해 보행기의 위치제어
  >* 손잡이 부분의 Encoder, PSD센서의 값을 PD제어기에 활용한다.
4.   PID제어를 이용하여 보행기의 속도제어
  >* PD제어기를 통해 얻은 값들을 PID 속도제어기와 Cascade control 	형식으로 구현한다. 

## System Architecture
<p align="center"><img src="https://user-images.githubusercontent.com/56825900/103440790-fd5fa100-4c8b-11eb-9a0c-0abb8a5f7ffb.jpg" width="600px"></p>  


### Code Overview  
- main.c 에 기본적인 센서, cascade control system 관련 코드가 들어있습니다.
- EWARM IAR을 활용하여 코드를 작성하였습니다.
- STM32F401RE(Nucleo-F401RE)를 활용하였습니다.
- ioc 파일은 ST사 관련 IDE로 모두 열 수 있습니다

### Project scenario

1. PC를 통해 flag data(OpenCV, Yolo를 통해 인식된 데이터로, 주행 방향 및 여부 등을 담고 있다.)를 받는다
2. 각 제어기를 통해 위치, 속도 등을 제어한다.
3. 오르막과 내리막등 지형에 상관 없이, 사용자의 보행기에 가하는 하중에 상관 없이 cascade control system을 통해 제어한다.


## 프로젝트 결과

<p align="center"><img src="https://user-images.githubusercontent.com/56825900/103440792-fe90ce00-4c8b-11eb-8cf1-be61c8c2a91c.jpg" width="500px"></p>  
<p align="center"> 사전 모델링 </p>  

<p align="center"><img src="https://user-images.githubusercontent.com/56825900/103440791-fe90ce00-4c8b-11eb-810b-dfebcbe886ce.jpg" width="500px"></p>  
<p align="center"> 완성작 </p>  

<p align="center"><img src="https://user-images.githubusercontent.com/56825900/103440913-07ce6a80-4c8d-11eb-95bb-435bcf2e8ff3.gif" width="500px"></p>  
<p align="center"> 실제 주행 영상 </p>  

