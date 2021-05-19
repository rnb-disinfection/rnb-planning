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
    - plane 좌표계 정렬 시 - 차이 없음
    ```
    BKPIECEkConfigFine: 	256170.0 ms/100 = 2561.7 ms (110.218/10085.097)
    KPIECEkConfigFine: 	51639.0 ms/100 = 516.4 ms (45.206/4278.697)
    RRTkConfigFine: 	8345.3 ms/100 = 83.5 ms (40.361/254.023)
    BKPIECEkConfigBad: 	100390.1 ms/10 = 10039.0 ms (10020.31/10081.335)
    KPIECEkConfigBad: 	67244.2 ms/10 = 6724.4 ms (544.676/10235.968)
    RRTkConfigBad: 	100197.3 ms/10 = 10019.7 ms (10013.146/10033.654)
    ```
    - 로봇 좌표계까지 정렬 시
    ```
    BKPIECEkConfigFine: 	7481.8 ms/10 = 748.2 ms (112.028/1916.677)
    KPIECEkConfigFine: 	6193.5 ms/10 = 619.3 ms (59.914/1562.995)
    RRTkConfigFine: 	673.7 ms/10 = 67.4 ms (37.57/124.372)
    BKPIECEkConfigBad: 	100514.2 ms/10 = 10051.4 ms (10029.246/10087.536)
    KPIECEkConfigBad: 	78809.0 ms/10 = 7880.9 ms (115.464/10189.973)
    RRTkConfigBad: 	100183.0 ms/10 = 10018.3 ms (10012.292/10043.563)
    ```
    - 위치 이동 시 (singularity 벗어남)
    ```
    BKPIECEkConfigFine: 	34645.1 ms/20 = 1732.3 ms (71.716/7965.63)
    KPIECEkConfigFine: 	6206.0 ms/20 = 310.3 ms (51.943/1301.905)
    RRTkConfigFine: 	2056.1 ms/20 = 102.8 ms (42.116/197.287)
    BKPIECEkConfigBad: 	9200.0 ms/5 = 1840.0 ms (225.816/5248.372)
    KPIECEkConfigBad: 	5768.9 ms/5 = 1153.8 ms (318.394/1767.999)
    RRTkConfigBad: 	485.6 ms/5 = 97.1 ms (74.2/126.474)
    ```
    - 모든 변형 시나리오에서 경향성 동일하게 관찰
 
  - Virtual 결과 일관적이지 않음, 대량 실험 해서 경향성 세밀 분석
    - 3m 넓은 트랙, 90% inlier
    ```
    BKPIECEkConfigDefault - 364.211755329 ms  - 1
    KPIECEkConfigDefault - 1111.92541652 ms    - 3
    RRTkConfigDefault - 480.864795049 ms         - 2
    RRTConnectkConfigDefault:     5428.9 ms        - 4
    ```
    - 0.5m 좁은 트랙, 90% inlier
    ```
    BKPIECEkConfigDefault - 951.631975174 ms   - 3
    KPIECEkConfigDefault - 351.74937778 ms       - 2
    RRTkConfigDefault - 73.9153014289 ms          - 1
    RRTConnectkConfigDefault: 8024.9 ms             - 4
    ```
  - **잠정 결론:**
    - 넓은 평면에서는 BKPIECE가 효용이 있으나 좁은 면에선 KPIECE가 나음
    - KPIECE는 넓은 면에서 성능 저하가 심함
    - 종합적으로는 그냥 RRT-Connect는 좁은면, 넓은 면 모두 성능 매우 떨어짐
    - 종합적으로는 그냥 RRT가 안정적이게 우수
    
  - **추가 문제 발생 시 의문 사항:**
    - Bad 케이스 KPIECE로 만들어진 경로는 실제 동작도 유효한지? 불연속한 경로 아닌지?
    - 실험 코드의 구속 조건 자체에 오류?
 
 
### Normal motion planning instability problem
- 일반 RRT도 실패 케이스에 대해 RRT vs RRTConnect 비교 분석 필요
 
 
### False case inspection
- 그 외에 생각 못한 경우 파악 위해 Tried: False 개별 케이스 면밀히 분석
 