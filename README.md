# Mocap Drones

### A general purpose motion capture system built from the ground up, used to autonomously fly multiple drones indoors

## Runing the code
Run `yarn run install` to install node dependencies 

Then run `yarn run dev` to start the webserver. You will be given a url view the frontend interface.

In another terminal window, run `python3 api/index.py` to start the backend server. This is what receives the camera streams and does motion capture computations.

## Documentation
The documentation for this project is admittedly pretty lacking, if anyone would like to put the type definition in the Python code that would be amazing and probably go a long way to helping the readability of the code. Feel free to also use the [discussion](https://github.com/jyjblrd/Mocap-Drones/discussions) tab to ask questions.

## YouTube Video
Watch this for information about the project & a demo!
[https://youtu.be/0ql20JKrscQ?si=jkxyOe-iCG7fa5th](https://youtu.be/0ql20JKrscQ?si=jkxyOe-iCG7fa5th)
![](https://github.com/jyjblrd/Mocap-Drones/blob/main/images/thumbnail.png?raw=true)

## Architectural Diagram
![](https://github.com/jyjblrd/Mocap-Drones/blob/main/images/architecture.png?raw=true)
