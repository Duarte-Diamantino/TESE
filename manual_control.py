import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import sys
import tty
import termios

def getch():
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        return sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)

class ManualControl(Node):
    def __init__(self):
        super().__init__('manual_control')
        self.speed_pub = self.create_publisher(Float64, '/commands/motor/speed', 10)
        self.steer_pub = self.create_publisher(Float64, '/commands/servo/position', 10)
        self.timer = self.create_timer(0.1, self.loop)

        self.speed = 0.0
        self.servo = 0.5  # centro recalculado entre 0.150 e 0.850
        self.STEP = 1000.0
        self.SERVO_STEP = 0.02
        self.SERVO_MIN = 0.150
        self.SERVO_MAX = 0.850

        print("Controlo manual iniciado: W/S para velocidade, A/D para direção, X para reset, Q para sair")

    def loop(self):
        char = getch().lower()

        if char == 'w':
            self.speed += self.STEP
        elif char == 's':
            self.speed -= self.STEP
        elif char == 'a':
            self.servo += self.SERVO_STEP  # agora a tecla A vira para a direita
        elif char == 'd':
            self.servo -= self.SERVO_STEP  # agora D vira para a esquerda
        elif char == 'x':
            self.speed = 0.0
            self.servo = 0.5
        elif char == 'q':
            rclpy.shutdown()
            return

        self.servo = min(max(self.servo, self.SERVO_MIN), self.SERVO_MAX)
        self.publish()

    def publish(self):
        msg_speed = Float64()
        msg_speed.data = self.speed
        self.speed_pub.publish(msg_speed)

        msg_steer = Float64()
        msg_steer.data = self.servo
        self.steer_pub.publish(msg_steer)

        print(f"Velocidade: {self.speed:.1f}, Direção: {self.servo:.3f}")

def main():
    rclpy.init()
    node = ManualControl()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
