# eddiebot_description



This packages define the urdf model of every eddiebot component for generating `tf` information.



### Package Structure

- [urdf](https://github.com/TooSchoolForCool/CIESSL/tree/master/eddiebot/eddiebot_description/urdf) defined every component of the robot
- [robots](https://github.com/TooSchoolForCool/CIESSL/tree/master/eddiebot/eddiebot_description/robots) builds the robot by using components defined in [urdf](https://github.com/TooSchoolForCool/CIESSL/tree/master/eddiebot/eddiebot_description/urdf)



### View the Model

Try following command the view the model

```bash
roslaunch eddiebot_rviz_launchers view_model.launch
```

or your can specify the model you want to view by passing a parameter to the launch file

```bash
roslaunch eddiebot_rviz_launchers view_model.launch model:=<model_name>
```



Following models are provided:

- eddie_kinect
- eddie_primesense



Here is an example for the model `eddie_kinect`

![eddie-kinect-model-demo](assets/eddie-kinect-model-demo.png)