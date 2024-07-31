import time
import math
import asyncio
from enum import Enum

from InverseKinematics import InverseKinematics, interpolate, KeyframeKinematics

class Locomotion:
    """Main class for running keyframe animation for SpotMicro.

    This class defines the keyframe animation and provides access functions
    to control the animation for SpotMicro. The animations only define the
    position of the feet and use the inverse kinematic equasions to convert
    them into servo angles which are then sent to the servo controller.
    """
    class Leg(Enum):
        # Enum difinition for the four legs
        FrontLeft = 0
        FrontRight = 1
        BackLeft = 2
        BackRight = 3
    
    def __init__(self, servos):
        """Init function

        Parameters
        ----------
        servos : list
            list of 12 servos, ordered by leg: FrontLeft, FrontRight, BackLeft, BackRight,
            with the servos in each leg ordered by: shoulder, leg, foot.
        """
        self._servos = servos
        self._running = False
        self._keyframes = []
        for i in range(4):
            self._keyframes.append([[0,150, 0], [0,150, 0]])
            
        self._standing = True
        self._forward_factor = 0.0
        self._rotation_factor = 0.0
        self._lean_factor = 0.0
        self._height_factor = 0.0
        

    async def Run(self):
        """Main loop for running keyframe animation.
        
        When executed, this function will start the keyframe animation and continue until
        the Shutdown function is called. It can be run using asyncio to process other events.
        """
        elapsed = 0.0
        gait = [[-10.0, 150.0, 40.0], [-10.0,120.0,40.0], [10.0, 120.0, 40.0], [10.0, 150.0, 40.0], [3.5, 150.0, 40.0], [-3.5, 150, 40]]
        start = time.time()
        
        last_index = -1
        self._running = True
        while self._running:
            elapsed += (time.time()-start)*15
            if elapsed >= len(gait):
                elapsed -= len(gait)
            start = time.time()
            index = math.floor(elapsed)
            ratio = elapsed - index

            if last_index != index:
                self._shift_keyframes()
                
                if self._standing:
                    self._set_standing_keyframes()
                else:
                    angle = 45.0/180.0*math.pi
                    x_rot = math.sin(angle) * self._rotation_factor
                    z_rot = math.cos(angle) * self._rotation_factor

                    angle = (45+gait[index][0])/180.0*math.pi
                    x_rot = x_rot-math.sin(angle) * self._rotation_factor
                    z_rot = z_rot-math.cos(angle) * self._rotation_factor
                    self._keyframes[self.Leg.FrontRight.value][1] = [gait[index][0]*self._forward_factor+x_rot, gait[index][1], gait[index][2]+z_rot]
                    self._keyframes[self.Leg.BackLeft.value][1] = [gait[index][0]*self._forward_factor-x_rot, gait[index][1], gait[index][2]+z_rot]
    
                    adjusted_index = index+3
                    if adjusted_index >= len(gait): adjusted_index -= len(gait)

                    angle = 45.0/180.0*math.pi
                    x_rot = math.sin(angle) * self._rotation_factor
                    z_rot = math.cos(angle) * self._rotation_factor

                    angle = (45+gait[adjusted_index][0])/180.0*math.pi
                    x_rot = x_rot-math.sin(angle) * self._rotation_factor
                    z_rot = z_rot-math.cos(angle) * self._rotation_factor
                    self._keyframes[self.Leg.FrontLeft.value][1] = [gait[adjusted_index][0]*self._forward_factor-x_rot, gait[adjusted_index][1], gait[adjusted_index][2]-z_rot]
                    self._keyframes[self.Leg.BackRight.value][1] = [gait[adjusted_index][0]*self._forward_factor+x_rot, gait[adjusted_index][1], gait[adjusted_index][2]-z_rot]

                last_index = index

            self._InterpolateKeyframes(ratio)

            await asyncio.sleep(0)

        # Set keyframes for standing position
        self._shift_keyframes(elapsed - math.floor(elapsed))
        self._set_standing_keyframes()
        self._standing = True
            
        elapsed = 0.0
        start
        while elapsed < 1:
            elapsed += (time.time()-start)*20
            start = time.time()
            self._InterpolateKeyframes(elapsed)

            await asyncio.sleep(0)

    def set_forward_factor(self, factor):
        """Set forward and backward movement.

        Parameters
        ----------
        factor : float
            Positive values move forward, negative values move back. Should be in the range -1.0 - 1.0.
        """
        self._forward_factor = factor*2

    def set_rotation_factor(self, factor):
        """Set rotation movement.

        Parameters
        ----------
        factor : float
            Positive values rotate right, negative values rotate left. Should be in the range -1.0 - 1.0.
        """
        self._rotation_factor = factor*100
