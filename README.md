### whweeltec_robot_rc_my 

该项目为车型号s400的ros节点

首先在`upload.cmd`中配置一下ssh

```shell
Host rosr
	Hostname 192.168.0.100
	User root
	Port 22
	IdentityFile ~/.ssh/id_rsa
```

Hostname 为小车ip地址, 注意把公钥提前置于`/root/.ssh/authorized_keys`中

1. 点击`upload.cmd`上传当前文件
2. 进入到小车服务器执行`ros1`和`roslaunch wheeltec_robot_rc_my keyboard_teleop.launch`
3. 之后从`opentcs`下发指令即可