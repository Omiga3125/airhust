步骤：

roslaunch simulation sim.launch

指令：

rostopic info /cloud\_registered

rostopic type /cloud\_registered

rostopic echo -n 1 /cloud\_registered/header

rostopic hz /cloud\_registered

输出：

Type: sensor\_msgs/PointCloud2



Publishers: 

&#x20;\* /cloud\_register\_mapping (http://airhust:43211/)



Subscribers: None





sensor\_msgs/PointCloud2







seq: 569

stamp: 

&#x20; secs: 217

&#x20; nsecs:  25000000

frame\_id: "world"

\---





subscribed to \[/cloud\_registered]

WARNING: may be using simulated time

average rate: 10.000

&#x09;min: 0.090s max: 0.111s std dev: 0.00630s window: 8

average rate: 9.825

&#x09;min: 0.090s max: 0.121s std dev: 0.00839s window: 15

average rate: 9.920

&#x09;min: 0.086s max: 0.125s std dev: 0.00977s window: 22

...











