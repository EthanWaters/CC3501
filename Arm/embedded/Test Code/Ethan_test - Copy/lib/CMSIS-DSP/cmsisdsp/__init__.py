import cmsisdsp.version

from cmsisdsp_filtering import *
from cmsisdsp_matrix import *
from cmsisdsp_support import *
from cmsisdsp_statistics import *
from cmsisdsp_complexf import *
from cmsisdsp_basic import *
from cmsisdsp_controller import *
from cmsisdsp_transform import *
from cmsisdsp_interpolation import *
from cmsisdsp_quaternion import *
from cmsisdsp_fastmath import *
from cmsisdsp_distance import *
from cmsisdsp_bayes import *
from cmsisdsp_svm import *


__version__ = cmsisdsp.version.__version__

# CMSIS-DSP Version used to build the wrapper
cmsis_dsp_version="1.14.1"


# CMSIS-DSP Commit hash used to build the wrapper
commit_hash="b8177102d9a4aaf83fd3f067364ecfa3100966c2"

# True if development version of CMSIS-DSP used
# (So several CMSIS-DSP versions may have same version number hence the commit hash)
developmentVersion=False

__all__ = ["datatype", "fixedpoint", "mfcc"]

# Default values
DEFAULT_HOUSEHOLDER_THRESHOLD_F64=1.0e-16
DEFAULT_HOUSEHOLDER_THRESHOLD_F32=1.0e-12