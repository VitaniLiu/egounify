"""Hardware-free SE decoder and real ROS-message publication regression."""
from types import SimpleNamespace as NS
import unittest
from litchi_hardware.core.types import HandSide
from litchi_hardware.hardware.sharpa.tactile import decode_tactile_frame, SharpaFinger, TactileImage


class SeTests(unittest.TestCase):
    def frame(self):
        return decode_tactile_frame(dict(frame_id=7,ts=1.5,channel=0,
            content={'DEFORM_SE':bytes([17,31,53])*3600,'F6':[1,2,3,4,5,6]}),
            HandSide.RIGHT,SharpaFinger.THUMB)

    def test_se_rgb_pixels_and_wrench_survive_decode(self):
        frame=self.frame()
        self.assertIsNotNone(getattr(frame,'deformation_se',None),'SE image discarded')
        image=frame.deformation_se
        self.assertEqual((image.width,image.height,image.encoding,image.step),(60,60,'rgb8',180))
        self.assertEqual(image.data,bytes([17,31,53])*3600)
        self.assertEqual(frame.wrench,(1.,2.,3.,4.,5.,6.))
        self.assertIsNone(frame.raw)
        self.assertEqual(TactileImage(2,1,b'12').step,2)

    def test_malformed_se_bytes_are_rejected_not_silently_lost(self):
        with self.assertRaises(ValueError):
            decode_tactile_frame({'content':{'DEFORM_SE':bytes(10799)}},HandSide.RIGHT,SharpaFinger.THUMB)

    def test_node_publishes_se_without_any_control_callback(self):
        from sharpa_driver.node import SharpaDriverNode
        frame=self.frame();rows={k:[] for k in ('raw','deformation','deformation_se','wrench')}
        side=HandSide.RIGHT;finger=SharpaFinger.THUMB
        pubs={k:NS(publish=v.append) for k,v in rows.items()}
        from builtin_interfaces.msg import Time
        node=NS(_hands={side:NS(read_tactile=lambda *a,**kw:frame)},
                _tactile_publishers={(side,finger):pubs},_last_tactile_warning_at={side:0.},
                get_clock=lambda:NS(now=lambda:NS(to_msg=lambda:Time(sec=2))),
                get_logger=lambda:NS(error=lambda message:self.fail(message)))
        SharpaDriverNode._publish_tactile(node)
        self.assertEqual(len(rows['deformation_se']),1)
        image=rows['deformation_se'][0]
        self.assertEqual((image.encoding,image.step,len(image.data)),('rgb8',180,10800))
        self.assertEqual(image.header.frame_id,'right_thumb_fingertip')
        self.assertEqual(len(rows['wrench']),1)
        self.assertEqual(rows['raw'],[])

if __name__=='__main__':unittest.main()
