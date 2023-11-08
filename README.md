# Mocap Drones

### A general purpose motion capture system built from the ground up, used to autonomously fly multiple drones indoors

## Runing the code
Run `yarn run install` to install node dependencies 

Then run `yarn run dev` to start the webserver. You will be given a url view the frontend interface.

In another terminal window, run `python3 api/index.py` to start the backend server. This is what receives the camera streams and does motion capture computations.
