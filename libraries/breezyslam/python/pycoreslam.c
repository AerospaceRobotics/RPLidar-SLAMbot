/*
pycoreslam.c : C extensions for CoreSLAM in Python

This code is free software: you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License as 
published by the Free Software Foundation, either version 3 of the 
License, or (at your option) any later version.

This code is distributed in the hope that it will be useful,     
but WITHOUT ANY WARRANTY without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU Lesser General Public License 
along with this code.  If not, see <http://www.gnu.org/licenses/>.

Change log:

07-FEB-2014 : Simon D. Levy  - Initial release
17-FEB-2014 : SDL - Prohibit non-positive random seed
28-FEB-2014 : SDL - Check for null return in Robot.computeVelocities()
01-MAR-2014 : SDL - Moved module code to in __init__.py
14-MAR_2014 : SDL - Changed mm_per_pixel to pixels_per_meter
                  - No more robot object passed to __init__(); velocities 
                    sent directly into update()
16-MAR_2014 : SDL - Changed #include to ../coreslam
23-MAR-2014 : SDL - Changed millimeters to meters
31-MAR-2014 : SDL - Improved documetnation for Laser class
                  - Made all units explicit

*/

#include <Python.h>
#include <structmember.h>

#include <math.h>

#include "../coreslam/coreslam.h"

// Python2 support
#if PY_MAJOR_VERSION < 3
#define PyInt_FromLong PyLong_FromLong
#define PyUnicode_AsUTF8 PyString_AsString
#endif

// Helpers ---------------------------------------------------------------------


static void
raise_argument_exception(const char * classname, const char *  methodname, const char * details)
{
    char errmsg[1000];
    sprintf(errmsg, "Wrong/insufficient arguments passed to %s.%s()%s %s", 
        classname, methodname, 
        details ? ":" : "", details ? details : "");
    PyErr_SetString(PyExc_TypeError, errmsg);
    
}

static PyObject *
null_on_raise_argument_exception_with_details(const char * classname, const char *  methodname, const char * details)
{
    raise_argument_exception(classname, methodname, details);
    return NULL;
}

static PyObject *
null_on_raise_argument_exception(const char * classname, const char *  methodname)
{
    return null_on_raise_argument_exception_with_details(classname, methodname, NULL);
}

static int
error_on_raise_argument_exception_with_details(const char * classname, const char *  methodname, const char * details)
{
    raise_argument_exception(classname, methodname, details);
    return -1;
}

static int
error_on_raise_argument_exception(const char * classname, const char *  methodname)
{
    return error_on_raise_argument_exception_with_details(classname, methodname, NULL);
}

static PyObject *
safe_get_attr_string(PyObject * obj, const char * attr_name, 
    const char * classname,  const char * method_name, const char * arg_name)
{
    PyObject * attr =  PyObject_GetAttrString(obj, attr_name);
    
    if (!attr)
    {
        char errstr[200];
        sprintf(errstr, "%s object does not have %s attribute", arg_name, attr_name);
        raise_argument_exception(classname, method_name, errstr);
    }
    
    return attr;
}

// returns 0 on success, 1 on failure
static int
double_from_object(double * result,
    PyObject * obj, const char * attr_name, 
    const char * classname,  const char * method_name, const char * arg_name)
{
    PyObject * attr_value = safe_get_attr_string(obj, attr_name, 
        classname, method_name, arg_name);
    
    if (attr_value)
    {
        *result = PyFloat_AsDouble(attr_value);
        return 0;
    }
    
    return 1;
}

// returns 0 on success, 1 on failure
static int 
int_from_object(int * result,
    PyObject * obj, const char * attr_name, 
    const char * classname,  const char * method_name, const char * arg_name)
{
    PyObject * attr_value = safe_get_attr_string(obj, attr_name, 
        classname, method_name, arg_name);
    
    if (attr_value)
    {
        *result = PyLong_AsLong(attr_value);
        return 0;
    }
    
    return 1;
}

static double
double_from_tuple(PyObject * tuple, int pos)
{
    return PyFloat_AsDouble(PyTuple_GetItem(tuple, pos));
}

static void
double_to_tuple(PyObject * tuple, double val, int pos)
{
    PyTuple_SetItem(tuple, pos, PyFloat_FromDouble(val));
}

/*
static int pyobj2position(PyObject * obj, position_t * pos)
{
    if (PyTuple_Check(obj) && PyTuple_GET_SIZE(obj) == 3)
    {        
        pos->x = double_from_tuple(obj, 0);
        pos->y = double_from_tuple(obj, 1);
        pos->theta = double_from_tuple(obj, 2);
        
        return 1;
    }
    
    return 0;
}
*/

static void position2tuple(position_t position, PyObject * tuple)
{
    
    double_to_tuple(tuple, position.x, 0);
    double_to_tuple(tuple, position.y, 1);
    double_to_tuple(tuple, position.theta, 2);
}

// CoreSLAM class ------------------------------------------------------------

typedef struct 
{
    PyObject_HEAD
    
    // State of the entire system
    state_t state;
    
    // Passed into update()
    int * lidar;
    
    // Settable instance variables
    int map_quality;
    double hole_width_meters;
    double sigma_xy_meters;
    double sigma_theta_degrees;    
    
} CoreSLAM;

static void
CoreSLAM_dealloc(CoreSLAM* self)
{    
    state_free(&self->state);
    
    Py_TYPE(self)->tp_free((PyObject*)self);
}

static PyObject *
CoreSLAM_new(PyTypeObject *type, PyObject *args, PyObject *kwds)
{    
    CoreSLAM *self;
    
    self = (CoreSLAM *)type->tp_alloc(type, 0);
    
    return (PyObject *)self;
}

static int
CoreSLAM_init(CoreSLAM *self, PyObject *args, PyObject *kwds)
{        
    self->map_quality = DEFAULT_MAP_QUALITY;
    self->hole_width_meters = DEFAULT_HOLE_WIDTH_METERS;   
    
    self->sigma_xy_meters = DEFAULT_SIGMA_XY_METERS;
    self->sigma_theta_degrees = DEFAULT_SIGMA_THETA_DEGREES;
    PyObject * py_laser = NULL;
    
    int map_size = 0;
    double map_scale = 0;
    
    PyObject * py_random_seed = NULL;
    
    static char * kwlist[] = {
        "laser", 
        "map_size", 
        "map_scale", 
        "random_seed",
    NULL};
    
    if (!PyArg_ParseTupleAndKeywords(args, kwds, "Oid|O", kwlist,
        &py_laser, 
        &map_size, 
        &map_scale, 
        &py_random_seed))
    {
        return error_on_raise_argument_exception("CoreSLAM", "__init__");
    }
    
    // Handle optional random-seed argument
    
    int random_seed = -1; // default -1 will tell coreslam_init to use time()
    
    if (py_random_seed != NULL)
    {
        random_seed = PyLong_AsLong(py_random_seed);
        
        if (random_seed < 1)
        {
            raise_argument_exception("CoreSLAM", "__init__", "random_seed must be positive");
        }
    }
            
    // Extract laser parameters for this CoreSLAM object, raising an exception on failure
    if (
        double_from_object(&self->state.laser.offset_meters, py_laser, "offset_meters", "CoreSLAM", "__init__", "laser") |
        int_from_object(&self->state.laser.scan_rate, py_laser, "scan_rate_hz", "CoreSLAM", "__init__", "laser") |
        int_from_object(&self->state.laser.angle_min, py_laser, "angle_min_degrees", "CoreSLAM", "__init__", "laser") |
        int_from_object(&self->state.laser.angle_max, py_laser, "angle_max_degrees", "CoreSLAM", "__init__", "laser") |
        int_from_object(&self->state.laser.detection_margin, py_laser, "detection_margin", "CoreSLAM", "__init__", "laser") |
        double_from_object(&self->state.laser.distance_no_detection_meters, py_laser, "distance_no_detection_meters", "CoreSLAM", "__init__", "laser") |
        int_from_object(&self->state.laser.scan_size, py_laser, "scan_size", "CoreSLAM", "__init__", "laser")
        )
    {
        return 1;
    }
    
    // Allocate memory for LIDAR data
    if (!(self->lidar = (int *)malloc(self->state.laser.scan_size * sizeof(int))))
    {
        fprintf(stderr, "Unable to allocate memory for LIDAR data");
        return 1;
    }
    
    state_init(
        &self->state, 
        map_size, 
        map_scale, 
        random_seed);
        
    return 0;
}


static PyObject *
CoreSLAM_str(CoreSLAM *obj)
{        
    // PyString_FromFormat doesn't support %f, so we need a big string to use with sprintf
    char str[2000];
    
    char * fmt = "CoreSLAM: HOLE_WIDTH = %f m" 
    "\n          SIGMA_XY = %-6.0f m" 
    "\n          SIGMA_THETA = %-6.0f deg" 
    "\n          map.size=%dx%d pixels map.quality=%d/255 map.scale=%f pixels/m"
    "\n          position=<x=%6.3f m y=%6.3f m theta=%+6.3f deg>";
    
    state_t state = obj->state;
    
    position_t position = state.position_history[0];
    
    sprintf(str, fmt,
        obj->hole_width_meters, 
        obj->sigma_xy_meters,
        obj->sigma_theta_degrees,
        state.map_size, state.map_size, obj->map_quality, state.map_scale,
        position.x/1000, position.y/1000, position.theta);       
    
    return  PyUnicode_FromString(str);
}

static PyObject *
CoreSLAM_update(CoreSLAM * self, PyObject * args, PyObject * kwds)
{                
    PyObject * lidar = NULL;
    PyObject * velocities = Py_None;
    
    // Bozo filter on input arguments
    if (!PyArg_ParseTuple(args, "O|O", &lidar, &velocities))
    {
        return null_on_raise_argument_exception("CoreSLAM", "update");
    }
    
    // Bozo filter on LIDAR argument
    if (!PyList_Check(lidar))
    {
        return null_on_raise_argument_exception_with_details("CoreSLAM", "update", "lidar must be a list");
    }
    
    laser_t laser = self->state.laser;
    
    // Bozo filter on LIDAR argument list size
    if (PyList_Size(lidar) != laser.scan_size)
    {
        return null_on_raise_argument_exception_with_details("CoreSLAM", "update", "lidar size mismatch");
    }
    
    // Extract LIDAR values from argument
    int k = 0;
    for (k=0; k<laser.scan_size; ++k)
    {
        self->lidar[k] = PyFloat_AsDouble(PyList_GetItem(lidar, k));
    }
        
    // Initialize velocities (not used on first iteration)
    double dxy_meters = 0;
    double dtheta_degrees = 0;
    double dt_seconds = 0;
    
    // Get velocities if indicated
    if (velocities != Py_None)
    { 
        if (PyTuple_Check(velocities) && PyTuple_GET_SIZE(velocities) == 3) {
            dxy_meters = double_from_tuple(velocities, 0);
            dtheta_degrees = double_from_tuple(velocities, 1);
            dt_seconds = double_from_tuple(velocities, 2);
        }
        else
        {
            return null_on_raise_argument_exception_with_details("CoreSLAM", "update", 
                "velocities must be (dxy_meters, dtheta_degrees, dt_seconds)");
        }
    }
    
    // Update state
    position_t pos = 
    state_update(
        &self->state, 
        self->lidar,
        dxy_meters, 
        dtheta_degrees,
        dt_seconds,
        self->map_quality, 
        self->hole_width_meters, 
        self->sigma_xy_meters,
        self->sigma_theta_degrees);
    
    PyObject * tuple = PyTuple_New(3);
    
    position2tuple(pos, tuple);    
       
    return tuple;
}


static PyObject *
CoreSLAM_getmap(CoreSLAM * self, PyObject * args, PyObject * kwds)
{
    PyObject * mapbytes = NULL;
    
    if (!PyArg_ParseTuple(args, "O", &mapbytes))
    {
        return null_on_raise_argument_exception("CoreSLAM", "getmap");
    }
    
    if (!PyByteArray_Check(mapbytes))
    {
        return null_on_raise_argument_exception_with_details("CoreSLAM", "getmap", "argument is not a byte array");
        
    }
    
    if (PyByteArray_GET_SIZE(mapbytes) != (self->state.map_size * self->state.map_size))
    {
        return null_on_raise_argument_exception_with_details("CoreSLAM", "getmap", "mapbytes are wrong size");
    }
    
    state_get_map(&self->state, PyByteArray_AsString(mapbytes));
    
    Py_RETURN_NONE;
}

static PyObject *
CoreSLAM_setmap(CoreSLAM * self, PyObject * args, PyObject * kwds)
{
    PyObject * mapbytes = NULL;
    
    if (!PyArg_ParseTuple(args, "O", &mapbytes))
    {
        return null_on_raise_argument_exception("CoreSLAM", "getmap");
    }
    
    if (!PyByteArray_Check(mapbytes))
    {
        return null_on_raise_argument_exception_with_details("CoreSLAM", "getmap", "argument is not a byte array");
        
    }
    
    if (PyByteArray_GET_SIZE(mapbytes) != (self->state.map_size * self->state.map_size))
    {
        return null_on_raise_argument_exception_with_details("CoreSLAM", "getmap", "mapbytes are wrong size");
    }
    
    state_set_map(&self->state, PyByteArray_AsString(mapbytes));
    
    Py_RETURN_NONE;
}

static PyObject *
CoreSLAM_getcloud(CoreSLAM * self, PyObject * args, PyObject * kwds)
{
    PyObject * cloudlist = PyList_New(self->state.cloud_size);
    
    int k;
    for (k=0; k<self->state.cloud_size; ++k)
    {     
        PyObject * tuple = PyTuple_New(4);      
        cloudpoint_t cloudpoint = self->state.point_cloud[k];       
        position2tuple(cloudpoint.position, tuple);        
        double_to_tuple(tuple, cloudpoint.likelihood, 3);       
        PyList_SetItem(cloudlist, k, tuple);
    }
    
    return cloudlist;
}

static PyMethodDef CoreSLAM_methods[] = {
    {"update", (PyCFunction)CoreSLAM_update, METH_VARARGS,
        "update(lidar, velocities=None) -> (x_meters, y_meters, theta_degrees)\n"
        "Updates the positition and map based on the lidar scan and optional\n" \
        "velocities tuples (dxy_meters, dtheta_degrees, dt_seconds).\n" \
    "Returns the robot's new position."},
    {"getmap", (PyCFunction)CoreSLAM_getmap, METH_VARARGS, 
        "getmap(mapbytes)\n" \
        "Puts current map values into bytearray mapbytes, which should of of\n" \
    "size map_size * map_size."},
    {"setmap", (PyCFunction)CoreSLAM_setmap, METH_VARARGS, 
        "setmap(mapbytes)\n" \
    "Puts bytearray of size map_size * map_size into current map"},
    {"getcloud", (PyCFunction)CoreSLAM_getcloud, METH_NOARGS, 
        "getcloud()\n" \
    "Returns a list of pointcloud tuples (x_meters, y_meters, theta_degrees, likelihood)\n"},
    {NULL}  // Sentinel 
};

static PyMemberDef CoreSLAM_members[] = {
    {"MAP_QUALITY", T_INT, offsetof(CoreSLAM, map_quality), 0,
    "Integration speed of scans into map, in interval [0,255];\n default = 50"},
    {"HOLE_WIDTH_METERS", T_DOUBLE, offsetof(CoreSLAM, hole_width_meters), 0,
    "Width of holes to dig into map, in meters;\n default = 0.6"},
    {"SIGMA_XY_METERS", T_DOUBLE, offsetof(CoreSLAM, sigma_xy_meters), 0,
    "Standard deviation for X,Y position coordinates in particle-filter search;\n default = 0.1"},        
    {"SIGMA_THETA_DEGREES", T_DOUBLE, offsetof(CoreSLAM, sigma_theta_degrees), 0,
    "Standard deviation for rotation angle theta in particle-filter search;\n default = 20"},        
    {NULL}  /* Sentinel */
};

#define TP_DOC \
"Supports essential CoreSLAM operations.\n\n" \
"Based on\n\n" \
"@inproceedings{coreslam-2010,\n" \
"  author    = {Bruno Steux and Oussama El Hamzaoui},\n" \
"  title     = {CoreSLAM: a SLAM Algorithm in less than 200 lines of C code},\n" \
"  booktitle = {11th International Conference on Control, Automation, \n" \
"               Robotics and Vision, ICARCV 2010, Singapore, 7-10\n" \
"               December 2010, Proceedings},\n" \
"  pages     = {1975-1979},\n" \
"  publisher = {IEEE},\n" \
"  year      = {2010}\n\n" \
"Usage:\n\n" \
"CoreSLAM(laser, map_size_pixels, map_scale_pixels_per_meter,\n" \
"         random_seed=None)\n\n" \
"where:\n\n" \
"laser is an object with the attributes\n" \
"      scan_size                     number of rays per scan\n" \
"      scan_rate_hz                  laser scan rate in Hertz\n" \
"      angle_min_degrees             minimum laser angle in degrees\n" \
"      angle_max_degrees             maximum laser angle in degrees\n" \
"      distance_no_detection_meters  scan distances above this are treated as infinfity\n" \
"      detection_margin              number of rays at edges of scan to ignore\n" \
"      offset_meters                 forward/backward offset of laser motor from robot center\n"

static PyTypeObject coreslam_CoreSLAMType = 
{
    #if PY_MAJOR_VERSION < 3
    PyObject_HEAD_INIT(NULL)
    0,                                          // ob_size
    #else
    PyVarObject_HEAD_INIT(NULL, 0)
    #endif
    "pycoreslam.CoreSLAM",                      // tp_name
    sizeof(CoreSLAM),                           // tp_basicsize
    0,                                          // tp_itemsize
    (destructor)CoreSLAM_dealloc,               // tp_dealloc
    0,                                          // tp_print
    0,                                          // tp_getattr
    0,                                          // tp_setattr
    0,                                          // tp_compare
    (reprfunc)CoreSLAM_str,                     // tp_repr
    0,                                          // tp_as_number
    0,                                          // tp_as_sequence
    0,                                          // tp_as_positionping
    0,                                          // tp_hash 
    0,                                          // tp_call
    (reprfunc)CoreSLAM_str,                     // tp_str
    0,                                          // tp_getattro
    0,                                          // tp_setattro
    0,                                          // tp_as_buffer
    Py_TPFLAGS_DEFAULT | Py_TPFLAGS_BASETYPE,   // tp_flags
    TP_DOC,                                     // tp_doc 
    0,                                          // tp_traverse 
    0,                                          // tp_clear 
    0,                                          // tp_richcompare 
    0,                                          // tp_weaklistoffset 
    0,                                          // tp_iter 
    0,                                          // tp_iternext 
    CoreSLAM_methods,                           // tp_methods 
    CoreSLAM_members,                           // tp_members 
    0,                                          // tp_getset 
    0,                                          // tp_base 
    0,                                          // tp_dict 
    0,                                          // tp_descr_get 
    0,                                          // tp_descr_set 
    0,                                          // tp_dictoffset 
    (initproc)CoreSLAM_init,                    // tp_init 
    0,                                          // tp_alloc 
    CoreSLAM_new,                               // tp_new 
};

// pycoreslam module ------------------------------------------------------------


#if PY_MAJOR_VERSION >= 3
static PyModuleDef pycoreslammodule = 
{
    PyModuleDef_HEAD_INIT,
    "pycoreslam",
    NULL,
    -1,     // m_size
    NULL,   // m_methods
    NULL,   // m_reload
    NULL,   // m_traverse
    NULL,   // m_clear
    NULL    // m_free
};
#endif

static int 
_type_unready(PyTypeObject * type)
{
    return (PyType_Ready(type) < 0);
}

static void 
_add_class(PyObject * module, PyTypeObject * type, const char * name)
{
    Py_INCREF(type);
    PyModule_AddObject(module, name, (PyObject *)type);
    
}


#if PY_MAJOR_VERSION < 3

static PyMethodDef module_methods[] = 
{
    {NULL}  // Sentinel 
};

PyMODINIT_FUNC
initcoreslam(void) 
{    
    if (_type_unready(&coreslam_CoreSLAMType))
    {
        return;
    }
    
    PyObject * module = Py_InitModule3("coreslam", module_methods, NULL);
    
    
    if (module == NULL)
    {
        return;
    }
    
    _add_class(module, &coreslam_CoreSLAMType, "CoreSLAM");
}

#else

PyMODINIT_FUNC
PyInit_coreslam(void) 
{    
    if (_type_unready(&coreslam_CoreSLAMType))
    {
        return NULL;
    }
    
    coreslam_CoreSLAMType.tp_new = PyType_GenericNew;
    
    PyObject* module = PyModule_Create(&pycoreslammodule);
    
    if (module == NULL)
    {
        return NULL;
    }
    
    _add_class(module, &coreslam_CoreSLAMType, "CoreSLAM");
    
    return module;
}

#endif


