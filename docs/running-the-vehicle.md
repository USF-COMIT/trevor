# Running the Vehicle



## Setup Your SSH keys (first time only)



## Network Testing

```
ping trevor1.local
```

You can also talk to trevor directly by IP using the address:  10.1.1.39

You can also view the network status through the router interface [http://10.1.1.1](https://10.1.1.1/#Dashboard)

## Running the vehicle software

First begin by SSHing to the vehicle

```bash
ssh trevor@trevor1.local
```

Then simply launch the vehicle software with the top-level launch script

```bash
ros2 launch trevor go_trevor.launch.py
```

You can see what processes that script started with&#x20;

```bash
ros2 node list
```

## Shore side launch

You can launch the shore side processes with

```bash
ros2 launch trevor shore.launch.py
```

To access the ros inspection tools you can run

```
ros2 run rqt_gui rqt_gui
```

and select the plugin that you want to view.  &#x20;
