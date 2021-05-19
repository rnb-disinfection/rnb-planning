## DeveloperNote

### Cases of inefficient task sequence
* Move back to home before doing next task


### Movable Binder Held by Robot
* error due to actor_robot_dict


### Note Info
* registering planner implementations: registerDefaultPlanners


### Constrained motion planning instability problem
#### with virtual plane
```
BKPIECEkConfigDefault:     30617.1 ms/100 = 306.2 ms (45.038/900.671)
KPIECEkConfigDefault:     40873.6 ms/100 = 408.7 ms (19.622/1969.107)
RRTkConfigDefault:     43292.3 ms/100 = 432.9 ms (18.505/2624.852)
RRTConnectkConfigDefault:     27144.7 ms/5 = 5428.9 ms (251.446/7565.886)
LBKPIECEkConfigDefault:     43063.5 ms/5 = 8612.7 ms (5521.325/10175.108)
```
- BKPIECE < KPIECE < RRT

#### with detected plane
```
BKPIECEkConfigFine: 	226720.7 ms/100 = 2267.2 ms (69.937/10082.002)
KPIECEkConfigFine: 	44196.6 ms/100 = 442.0 ms (46.311/1800.006)
RRTkConfigFine: 	9627.6 ms/100 = 96.3 ms (39.244/344.946)
BKPIECEkConfigBad: 	100600.4 ms/10 = 10060.0 ms (10027.311/10100.471)
KPIECEkConfigBad: 	77786.7 ms/10 = 7778.7 ms (418.654/10314.316)
RRTkConfigBad: 	100182.4 ms/10 = 10018.2 ms (10011.62/10033.246)
```
 - Fine 케이스에서 RRT < KPIECE < BKPIECE, Bad 케이스에서 KPIECE < RRT,BKPIECE로 virtual 실험과 다른 경향
 - 좌표계와 정렬이 안된 게 퍼포먼스에 영향?
 - 실험 코드의 구속 조건 자체에 오류?
 
 
### Normal motion planning instability problem
- 일반 RRT도 실패 케이스에 대해 RRT vs RRTConnect 비교 분석 필요
 
 
### False case inspection
- 그 외에 생각 못한 경우 파악 위해 Tried: False 개별 케이스 면밀히 분석
 