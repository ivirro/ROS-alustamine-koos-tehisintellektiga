# ROS-alustamine-koos-tehisintellektiga

Suurte keelemudelite kasutamine arendamiseks. Nende rakendamsiega saab alljärgnevat kasu.
1. Kiirem programeerimise voog.
2. Kiirem programmi silumine koos parendusega.
3. Kiirem arusaam koodi käitumisest.

Suurte keeltemudelite (LLM – Large lanuge model) põhineb Generative Pre-trained Transformer tehnoloogial (GPT). Lisaks on nendega suheldes kasutuse ka mitmeid teisi töötlussamme nende vastuste kuva vahel.

 Neid mudeleid on erinevaid. Üks tuntud on chatGPT https://chatgpt.com/ mida saab kasutada ka https://copilot.microsoft.com/ vahendusel. Sõltuvalt mudelist saab suhelda sellega kasutades kas inglise keelt või valimiku ka muust keeltest. Lisaks erinevad need kas saab lohistada pilte sisendiks või saada ka faile väljundiks. Mõnda saab jooksutada kohaliku raali peal ja teistega masinsuhtlemiseks on API võimalik kasutada.

Mõned alternatiivid:

https://claude.ai/

https://gemini.google.com/app

https://www.perplexity.ai/

https://notebooklm.google/

https://www.llama.com/


Liikudes sedasi näitele kuids programeelimis voogu kiirendada.
Kopeerida promt tekst näiteks chatGPT-le
```
Write a program for ROS2 in Python to move a robot forward during 5 seconds and then stop.
```

Tulemuseks peaks tulema midagi sarnast, mis vastab levinud ROS tavale.

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class ForwardMotion(Node):
    def __init__(self):
        super().__init__('forward_motion')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer_period = 0.1  # seconds
        self.timer = self.create_timer(self.timer_period, self.publish_motion)
        self.duration = 5.0  # 5 seconds
        self.start_time = time.time()

    def publish_motion(self):
        msg = Twist()
        msg.linear.x = 0.2  # Linear velocity (0.2 m/s)
        if time.time() - self.start_time >= self.duration:
            msg.linear.x = 0.0  # Stop the robot
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    forward_motion = ForwardMotion()
    rclpy.spin(forward_motion)
    forward_motion.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

Selle katsetamise jaoks saab kasutada The Construct kaugtöölauda.
https://app.theconstruct.ai/open-classes/52382476-423a-4f84-bf09-ad120c998719

## Kuidas simuleerimist käivitada

Simulatsiooni käivitamiseks tuleb esmalt oma tööruum allikaks määrata:

- Ava terminaliaknas, klõpsates oma ekraani vasakus allnurgas olevat terminaliikooni:


- Kopeeri ja kleebi järgmised käsud oma terminali:

<span class="badge badge-pill badge-primary">
    <i class="fa fa-play"></i>
    &nbsp;
    Käivita terminalis
</span>

```python
cd ~/ros2_ws/src/neobotix_ros2
./start_basic_neobotix_sim.sh
```

Kui lähed oma ekraani vasakus allnurgas Gazebo nupu juurde, peaksid nägema simulatsiooni ning saama juhtida seda nii, nagu see oleks päris robot:

Oota maksimaalselt umbes 30 sekundit, kuni simulatsioon käivitub, ja sa peaksid nägema simulatsiooni.

Nüüd saad eelnevalt genereeritud koodi lisada projketi ja proovida, millise tulemuse see annab.


Järgmiseks silumise näide.

```
Find bugs in this ROS2 node:
```
```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class ForwardMotion(Node):
    def __init__(self):
        super().__init__('forward_motion')
        self.publisher_ = self.create_publisher(Twist, 'robot_cmd_vel', 10)
        self.timer_period = 0.1  # seconds
        self.timer = self.create_timer(self.timer_period, self.publish_motion)
        self.duration = 5.0  # 5 seconds
        self.start_time = time.time()

    def publish_motion(self):
        msg = Twist()
        linear.x = 0.2  # Linear velocity (0.2 m/s)
        
        # Check if the duration has elapsed
        if time.time() - self.start_time >= self.duration:
            msg.linear.x = 0.0  # Stop the robot
        
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    forward_motion = ForwardMotion()
    rclpy.spin(forward_motion)
    forward_motion.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

Saad väljundiks midagi järgnevat:
```
# ROS2 Node Debugging and Fixes

This document provides an analysis of issues found in a ROS2 Python node and presents a corrected version of the code.

---

## Identified Bugs and Fixes

### 1. **Incorrect Attribute Access in `publish_motion`**
   **Problem**:  
   In the `publish_motion` method, the code references `linear.x` instead of `msg.linear.x`, which causes an error.

   **Fix**:  
   Replace:
   ```python
   linear.x = 0.2  # Linear velocity (0.2 m/s)
   ```
   ```python
   msg.linear.x = 0.2  # Linear velocity (0.2 m/s)
   ```
 Publisher Object Name
Problem:
The publisher is created as self.publisher_, but in the publish_motion method, the non-existent self.publisher is used, causing a runtime error.

Fix:
Update the reference to:
 ```python
self.publisher_.publish(msg)
 ```
3. Duration Logic
Problem:
The timer continues to run after the desired duration of 5 seconds has elapsed, creating unnecessary overhead.

Fix:
Add logic to cancel the timer after the duration has passed:
 ```python
if time.time() - self.start_time >= self.duration:
    msg.linear.x = 0.0  # Stop the robot
    self.timer.cancel()  # Cancel the timer
 ```
4. Topic Name
Problem:
The topic name robot_cmd_vel might not match the expected topic name used by the robot or simulation. The standard name is typically /cmd_vel.

Fix:
Update the topic name:
 ```python
self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
```
5. Optional Improvement: Duration Logic
Instead of using time.time() for duration management, you can consider using ROS2’s built-in Timer mechanisms for more precise control.

Corrected ROS2 Node Code
 ```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class ForwardMotion(Node):
    def __init__(self):
        super().__init__('forward_motion')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer_period = 0.1  # seconds
        self.timer = self.create_timer(self.timer_period, self.publish_motion)
        self.duration = 5.0  # 5 seconds
        self.start_time = time.time()

    def publish_motion(self):
        msg = Twist()
        msg.linear.x = 0.2  # Linear velocity (0.2 m/s)

        # Check if the duration has elapsed
        if time.time() - self.start_time >= self.duration:
            msg.linear.x = 0.0  # Stop the robot
            self.timer.cancel()  # Cancel the timer

        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    forward_motion = ForwardMotion()
    rclpy.spin(forward_motion)
    forward_motion.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

Summary of Changes
Fixed incorrect attribute access for msg.linear.x.
Corrected publisher reference from self.publisher to self.publisher_.
Added a mechanism to cancel the timer after the specified duration.
Updated the topic name to the standard /cmd_vel.
This corrected node should now work properly in a ROS2 environment.
```

Koodi kiirem mõitlise jaoks saab koodi jupi kopeerida päringusse.
On võimalik leida githubist palju erinevaid katsetamise jaoks näiteks: https://github.com/ros2/demos

Võtame ette esimes genereeritud koodi ja sellel lisaks promt.
```
explain this ros2 code with added comments:
```
vastuseks saab midagi sellist, kuna tegemist on generatiivse protsessiga siis alati kattuvat vastust ei saada võrreldes eelneva päringuga.

```python
#!/usr/bin/env python3

# Importing the necessary ROS2 Python libraries
import rclpy
from rclpy.node import Node  # Node class to create a ROS2 node
from geometry_msgs.msg import Twist  # Twist message for linear and angular velocities
import time  # For timing the motion duration

# Create a class ForwardMotion that inherits from Node (ROS2 class)
class ForwardMotion(Node):
    def __init__(self):
        # Initialize the Node with a name 'forward_motion'
        super().__init__('forward_motion')

        # Create a publisher that will send Twist messages to the 'cmd_vel' topic (robot velocity command)
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)

        # Set the timer period (how often the `publish_motion` method will be called)
        self.timer_period = 0.1  # 0.1 seconds interval
        self.timer = self.create_timer(self.timer_period, self.publish_motion)

        # Define the duration for which the robot will move forward (in seconds)
        self.duration = 5.0  # 5 seconds

        # Store the time when the motion started, using time.time() to capture the start time
        self.start_time = time.time()

    # Method to publish velocity commands (for robot movement)
    def publish_motion(self):
        # Create a Twist message instance to define velocity values
        msg = Twist()

        # Set the linear velocity in the x direction (forward direction) to 0.2 meters per second
        msg.linear.x = 0.2  # Move the robot forward at 0.2 m/s

        # Check if the elapsed time has passed the specified duration
        if time.time() - self.start_time >= self.duration:
            # If the duration has passed, stop the robot (set velocity to 0)
            msg.linear.x = 0.0  # Stop the robot after 5 seconds
        
        # Publish the velocity command (Twist message)
        self.publisher_.publish(msg)

# The main function to initialize the ROS2 system and start the node
def main(args=None):
    # Initialize the ROS2 communication system
    rclpy.init(args=args)

    # Create an instance of the ForwardMotion class (this starts the node)
    forward_motion = ForwardMotion()

    # Start spinning the node to keep it alive and processing
    rclpy.spin(forward_motion)

    # Cleanup and shutdown the node
    forward_motion.destroy_node()
    rclpy.shutdown()

# Entry point of the script; this is executed when the script is run
if __name__ == '__main__':
    main()
 ```
