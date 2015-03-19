# What is QuadRover? #
This code is the result of my PhD research<sup>1</sup> on autonomous indoor quadrotor flight. QuadRover is a flight controller that runs on an Android smartphone (no root access required). There are two external pieces of hardware needed (aside from the quadrotor itself): 1) an arduino board with USB host to received motor commands from the phone and relay them on to the motors, and 2) a height sensor that interfaces with the arduino which then relays the readings on to the phone.

Actually, depending on what type of flying you are doing the height sensor may not be necessary. Some newer phones actually have a barometer which, in theory, should be able to provide sufficient height information for outdoor flight. In practice, though, the barometer is severely biased by the phone temperature so some sort of calibration procedure would be required. For indoor flight, of course, something else is needed.

I've only tested this on Galaxy S2, S3, and S4 but I haven't done anything specific for those phones so, aside from some re-tuning, the software should be capable of running an any "recent" phone. Curiously, while the S4 for the most part has better hardware the flight performance is worse. I've played with it quite a bit but the only explanation I can think of is that the operating system has some additional delays somewhere (I suspect in the USB comm).

Also I haven't tested a fresh install in a long time so if you've tried to install it and something doesn't work, feel free to let me know. Finally, I do make use of some C++/11 features. The android ndk uses the gcc compiler so these are supported there, but you may run into problems with the PC side software if you are using Visual Studio.

# A bit more detail #
At the lowest level is an attitude observer. I've put considerable effort into getting this working well, and it's to the point that for all my test scenarios (not acrobatic flight) it compares very well to a Vicon motion capture system as the "ground truth."

More difficult is trying to get an accurate estimation of the translation state. Much of my research, especially over the past year, has been working on algorithms to extract useful information related to velocity using the onboard camera and external height sensor. Position relative to a target is also done using the onboard camera. Being able to do this within the constraints of the onboard processing power is currently an active field of research.

With very accurate attitude information and reasonably accurate translation state information, controller design is pretty straightforward. I have the system to the point now that it can hover over a target without any external assistance. The phone extracts regions (MSERs) of interest automatically so should be usable in arbitrary environments as long as there is some texture to the scene.

# Pictures and Videos #
<img src='https://quadrover.googlecode.com/hg/images/quadFlight_small.JPG' width='300'>

<h2>Autonomous flight!</h2>
Here's a demonstration of autonomous flight. It's a couple months old, but since that time I've been busy rewriting a bunch of the code plus trying to find a job :)<br>
<br>
In the video, takeoff is done using external Vicon cameras for XY position since the onboard camera is too close to the ground. Shortly after takeoff you see the quad jump to the right, which is a manual command from me to get over the more interesting objects on the ground. After that it is completely autonomous, i.e. using only onboard processing with NO Vicon information. The feature tracking you see is done based on probability of match which was a major part of my thesis. This had the advantage of being more robust and quicker than standard feature matching. One of the biggest advantages is it does not require the computation and explicit matching of feature descriptors.<br>
<br>
<a href='http://www.youtube.com/watch?feature=player_embedded&v=gmL8WAvt6gc' target='_blank'><img src='http://img.youtube.com/vi/gmL8WAvt6gc/0.jpg' width='425' height=344 /></a><br>
<br>
<h1>Documentation</h1>
I'm in the process of doing basic documentation of the important classes. After graduation I'll try flush it a little better ... maybe ... if enough other people actually want to use it :)<br>
<br>
<sup>1</sup> Intelligent Control Systems Lab at Seoul National University with H. Jin Kim