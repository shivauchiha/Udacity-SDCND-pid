# **Finding Lane Lines on the Road** 

## Writeup Template

### You can use this file as a template for your writeup if you want to submit it as a markdown file. But feel free to use some other method and submit a pdf if you prefer.

---

**PID controller for self driving cars**

The goals / steps of this project are the following:
* Fill up the PID class
* Implement a PID control for steering values
* Use manual or twiddle tuning algorithm to tune hyper parameters.
* Coustamize the control system, In my case assisted breaking system based on PID was implemented for more stable and safe turn.
* Further tune the parameters to account for changing speeds.
* could achieve stability within (35-50MPH)


[//]: # (Image References)

[image1]: 1.png "simulator"

---

### Reflection

### 1. Theory

The PID controller acronym for proportional integral and defferential controller is a type of controller used in application of reducing errors by generating the necessary actuation input.This type of control is one of the simplest and most commonly used in various industrial applications.In the case of self driving cars, It performs the job aligning the car to a generated trajectory.
Now lets look into what PID entails in detail. 

#### a.Proportional Control
This part is primary driver of the controller that forces the system to take inversion action in response to the direction of error.This is modeled as 

'''
P = -Kp * cte
'''
Here Kp is a hyperparameter.As you can see mathematically as error increases the control input is reduced.Ideally, this alone should be enough.However there could be cases when the controller may come in towards the zero error condition quite fast, This will result in a slight overshoot. To account for this new error the controller will again reverse the polarity of signal. In general it would slowly diverge to minimum error condition after several oscillations if the error trajectory is a stable straight trajector.

In self driving car perspective,If there is a high gain there will be continous minor oscillations similar to bang-bang control.However when there is a suddent lane change it can react faster albeit with minimum osillations.A correct gain will balance faster convergence and minimum oscillations.It should also be noted that very high gain can sometimes make the car go around in circles.

#### b.Differential Control
This part is damper driver of the controller that forces the system to take slower approach in response as it finds the rate of error gets reduced .This is modeled as 

'''
D = -Kd * d(cte)
'''
Here Kd is a hyperparameter.Here to visualize how it works, Think of a error rate graph.If the P controller works, Thr graph will start from high value and go down.When you take the differentiation of this graph you will get a negative value.This value tends to have retardation effect on the P controller output.Think of it as using the rate to predict the future and respond to it. 

In self driving car perspective,High gain tends to minimize the oscillationbut takes longer time to converge to the minimum error.Thus an ideal Kd value would be a value that balances convergence and osillation 

#### c.Integral Control
This part is stable state error correction driver of the controller that forces the system to take converge further towards minimum error, if it find the controller signal input is causing a stagnant error and not converging towards minimum  .This is modeled as 

'''
I = -Ki * I(cte)
'''
Here Ki is a hyperparameter and I is integral function.Due to inherent system bias or suddent environmental change that may change the dynamics of system ,The controller may fail to converge fully and there will be a slight offset or residual error.Here integral component keeps track of past information,If the controller input is giving constant output with consisten residual error this will have slight additive effect to the output of propotional controller .

In self driving car perspective,High gain tends to osillate the system as controller will respond to even minor disturbances and at low gain the controller may never converge if it comes across a unanticipate change in the system.A good Ki value will balance both.
####Controller
The final controller is defined as
'''
steer = -Kp * cte -Kd * d(cte) -Ki * I(cte)
'''

Good explaination of PID: [https://www.youtube.com/watch?v=wkfEZmsQqiA&t=594s]
vehicle perspective : [https://www.youtube.com/watch?v=4Y7zG48uHRo&t=178s]
![alt text][image1]


### 2. Model

The pid class was filled with proper gain initalisation.This was followed by completion of update function where the cte,d(cte),i(cte) were calculated as follows:

'''
d_error = cte - p_error;
p_error = cte;
i_error += cte;
'''
The total error were calculated as following:

'''
-((p_error*Kp)+(d_error*Kd)+(i_error*Ki));
'''

The resulting steer_value from controller was limited to [-1,1] by a checking condition.

An assisted breaking model was implemented using PID in addition to steering PID.

Intially, I attempted to adjust the throttle as follows:
'''
Throttle = 0.5- absf(PID) ;
'''
The idea was that as error increases speed decreases thus reducing the chance of car falling outside the track.But the problem was even though it was able to prevent the intended situtaion, It also took time to steer the car back to minimum error. This combined with increased velocity when it approaches the centre of track creates a unstable oscillating effect.

Thus a second plan was concucted,
We already knew the steering value form the steering pid.The wheel rotates from [-1,1] with 0 beign the centre.Ideally if the pid steers to 0 it means the car has achieved stability, This means this is optical condition for driving in maximum velocity.As the steer wheel diverges from centre we want to reduce velocity as we dont want to fall of the track nor intercept the centre of road at faster speeds.
We use this formula to calculate traget speed.

'''
float Target_speed = 40*(1.0-abs(steer_value))+10;
'''

The speed error is calculated by
 
'''
pspeed.UpdateError(scte);
float throttle = pspeed.TotalError();
'''
This error is fed into a pid which controls the throttle.




### 3. Hyper parameter tuning

I started by first setting constant throttle to 0.45.I began by first setting the P controller gain to 0.1 slowly increased until an stable oscillation is scene in the first section of track just before the first corner.Then started D controller gain at 100 times the gain value of P and slowly adjusted it until osillation is reduced.This approach was found in given link :[https://robotics.stackexchange.com/questions/167/what-are-good-strategies-for-tuning-pid-loops].Then integral was set to a minimum value of 0.0001 as higher value was inducing un stability to the simulation.By intution it shouldn't be necessary as the simulation is a perfect world and there shouldn't be any bias driven steady state error.However noticed a slight increase in performance at turns thus kept it at 0.0001 instead of zero.

Now once steering PID is tuned , I removed the constant throttle and fed the throttle for throttle pid.Just like above procedure the gains were adjusted until maximum possible stability was seen.

final steer(PID) :(0.15,0.0001,1)
Final throttle(PID) :(0.5,0.0001,1.5)

### 4.Result

![alt text][image1]

After the tuning as seen in the pid.mkv video a  good stable driving was achieved with ocassional minor osillations . The break assistance system does a fine job of decelerating at appropritae points
such that it reduces the strain on the steer pid to align with lane.The model achieved best control at turns.The achieved speed varied from (35-48MPH).Another issue that was noticed is with the simulator itself.I believe the GPU and CPU spec of hardware affects the latency of communication between code and simulator.When OBS(CPU and GPU intensice) was used to record the simulation noticed a slight increase in osillation proabaly due to the code receiving error data from simulator for slight delay thus forcing PID to compensate for past delayed response.


### 5.Future

I went through twiddle and SGD algorithm for parameter tuning.Since, I was able to achieve a fairly stable system with manual tuning.I decided to submit the project as all rubrics are satisfied.
In account of less time have for my course end.I had to postpone this effort.But I am planning to complete rest of the course as fast as possible and come back to implement twiddle() and see if i can push the driving operation beyond 50MPH.

Thank You !
