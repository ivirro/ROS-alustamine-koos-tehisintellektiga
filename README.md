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


Liikudes sedasi näitele kuids programeelimis voogu kiirendafa.
```python
Write a program for ROS2 in Python to move a robot forward during 5 seconds and then stop.

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
