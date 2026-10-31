import pytest
from litchi_hardware.core.types import HandSide

from litchi_hardware.hardware.sharpa import tactile as mod

def test_official_se_sdk_key_is_decoded_not_silently_ignored():
    frame=mod.decode_tactile_frame({'content':{'DEFORM3_SE':bytes([7])*10800,'F6':[1,2,3,4,5,6]}},
                                  HandSide.LEFT,mod.SharpaFinger.THUMB)
    assert frame.deformation_se is not None
    assert (frame.deformation_se.width,frame.deformation_se.height,frame.deformation_se.encoding)==(60,60,'rgb8')
    assert frame.deformation_se.data==bytes([7])*10800
    assert frame.wrench==(1.,2.,3.,4.,5.,6.)

def test_se_bad_shape_is_rejected():
    with pytest.raises(ValueError,match='shape'):
        mod.decode_tactile_frame({'content':{'DEFORM3_SE':b'broken'}},HandSide.RIGHT,mod.SharpaFinger.THUMB)

def test_legacy_key_still_decodes():
    frame=mod.decode_tactile_frame({'content':{'DEFORM_SE':bytes(10800)}},HandSide.RIGHT,mod.SharpaFinger.THUMB)
    assert len(frame.deformation_se.data)==10800
