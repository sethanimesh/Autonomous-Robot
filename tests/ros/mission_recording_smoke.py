#!/usr/bin/env python3
"""Generated camera -> recorder -> isolated ROS playback integration check."""
import json
import os
from pathlib import Path
import subprocess
import signal
import sys
import tempfile
import time
ROOT=Path(__file__).resolve().parents[2];sys.path.insert(0,str(ROOT))
from scripts.diagnostics.replay_mission import isolated_environment,load_recording


def main():
    output=Path(sys.argv[1]).resolve();output.mkdir(parents=True,exist_ok=True)
    isolated_environment(output)
    import rclpy
    from sensor_msgs.msg import Image
    from std_msgs.msg import String
    from geometry_msgs.msg import Twist
    rclpy.init();n=rclpy.create_node('recording_generated_test')
    ip=n.create_publisher(Image,'/camera/image_raw',10)
    rp=n.create_publisher(String,'/robot_status',10)
    cp=n.create_publisher(Twist,'/cmd_vel',10)
    received=[];observed_commands=[]
    def frame(m):
        received.append(m.header.stamp.sec+m.header.stamp.nanosec/1e9)
        if playback[0]:
            command=Twist();command.linear.x=.456;cp.publish(command)
    n.create_subscription(Image,'/camera/image_raw',frame,10)
    n.create_subscription(Twist,'/cmd_vel',lambda m:observed_commands.append(m.linear.x),10)
    playback=[False];recording=output/'generated-run'
    child=subprocess.Popen([sys.executable,str(ROOT/'scripts/diagnostics/record_mission.py'),
        '--record-camera','--output',str(recording),'--seconds','60','--fps','5'],stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True)
    started=time.monotonic()
    interrupted=False
    try:
        while child.poll() is None and time.monotonic()-started<12:
            m=Image();m.header.frame_id='synthetic';m.header.stamp=n.get_clock().now().to_msg()
            m.height=24;m.width=32;m.encoding='bgr8';m.step=96;m.data=bytes([80,120,180])*24*32
            ip.publish(m);rp.publish(String(data=json.dumps(dict(status='ok',motion_active=False))))
            command=Twist();command.linear.x=.123;cp.publish(command)
            rclpy.spin_once(n,timeout_sec=.05);time.sleep(.05)
            captured=len(list((recording/'frames').glob('*.jpg')))
            if time.monotonic()-started>4 and captured>=8 and not interrupted:
                child.send_signal(signal.SIGINT);interrupted=True
        stdout,stderr=child.communicate(timeout=3)
        assert child.returncode==0,(stdout,stderr)
        _,manifest,events=load_recording(recording)
        assert manifest['frames']>=5,manifest
        assert manifest['status']=='complete' and manifest['stop_reason']=='interrupted',manifest
        received.clear();observed_commands.clear();playback[0]=True
        replay=subprocess.Popen([sys.executable,str(ROOT/'scripts/diagnostics/replay_mission.py'),str(recording),
            '--mode','ros','--report',str(output/'ros-replay.json')],stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True)
        # Supply readiness only after replay and this probe can discover each other.
        t=time.monotonic()
        while time.monotonic()-t<1.5:rclpy.spin_once(n,timeout_sec=.05)
        received.clear();observed_commands.clear();replay.stdin.write('\n');replay.stdin.flush()
        t=time.monotonic()
        while replay.poll() is None and time.monotonic()-t<12:rclpy.spin_once(n,timeout_sec=.05)
        stdout,stderr=replay.communicate(timeout=3)
        assert replay.returncode==0,(stdout,stderr)
        result=json.loads((output/'ros-replay.json').read_text())
        assert received and result['commands_observed'],(received,result)
        assert all(v==.456 for v in observed_commands),observed_commands
        assert all(abs(time.time()-stamp)<10 for stamp in received),received
        result=dict(recorded_frames=manifest['frames'],replayed_frames_received=len(received),
            new_commands_logged=len(result['commands_observed']),recorded_commands_not_republished=True,
            timestamps_fresh=True,clean_interrupt_exit=True,source='generated_pixels_only',physical_ev3_used=False)
        (output/'smoke-result.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result))
    finally:
        for p in (child,locals().get('replay')):
            if p and p.poll() is None:p.terminate();p.wait(timeout=3)
        n.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
