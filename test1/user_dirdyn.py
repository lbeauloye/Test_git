# -*- coding: utf-8 -*-
"""Module for the definition of functions related to Equilibrium analysis."""
# Author: Robotran Team
# (c) Universite catholique de Louvain, 2020

import MBsysPy as Robotran
from MBsysPy.mbs_utilities import set_output
import numpy as np

def user_dirdyn_init(mbs_data, mbs_dirdyn):
    """Run specific operation required by the user before running direct dynamic.

    Parameters
    ----------
    mbs_data : MBsysPy.MbsData
        The instance containing the multibody project.
    mbs_dirdyn : MBsysPy.MbsDirdyn
        The instance of the current direct dynamic process.

    Returns
    -------
    None.

    """

    # Example: Creating and storing a sensor in mbs_data then create a list to store
    #          the vertical velocity of the sensor. Two new fields are added to
    #          the MbsData instance to store the sensor and the list.
    #
    # import MBsysPy as Robotran # Should be done outside the function.
    #
    # mbs_data.my_sensor = Robotran.MbsSensor(mbs_data)
    # mbs_data.my_sensor_v = []
    Robotran.define_output_vector('Qq', 1)
    mbs_data.sensor_passenger1 = Robotran.MbsSensor(mbs_data)
    Robotran.define_output_vector('qdd_sensor_passenger1', 3)
    
    id_sensor_nacelle1 = mbs_data.sensor_id['capteur_nacelle1']
    mbs_data.sensors.append(Robotran.MbsSensor(mbs_data, id_sensor_nacelle1))
    mbs_data.sensor_nacelle1 = Robotran.MbsSensor(mbs_data, id_sensor_nacelle1)
    Robotran.define_output_vector('q_sensor_nacelle1', 3)
    
    return


def user_dirdyn_loop(mbs_data, mbs_dirdyn):
    """Run specific operation required by the user at the end of each integrator step.

    In case of multistep integrator, this function is not called at intermediate
    steps.

    Parameters
    ----------
    mbs_data : MBsysPy.MbsData
        The instance containing the multibody project.
    mbs_dirdyn : MBsysPy.MbsDirdyn
        The instance of the current direct dynamic process.

    Returns
    -------
    None.
    """

    # Example: The sensor, created in `user_dirdyn_init`, is computed (fields
    #          are updated) and the vertical velocity is added in the list.
    #
    # mbs_data.my_sensor.comp_s_sensor(1)
    # mbs_data.my_sensor_v.append(mbs_data.my_sensor.V[3])
    id_sensor_passenger1 = mbs_data.sensor_id['sensor_passenger1']
    mbs_data.sensor_pasger1.comp_s_sensor(id_sensor_passenger1)
    id_sensor_nacelle1 = mbs_data.sensor_id['sensor_nacelle1']
    mbs_data.sensor_nacelle1.comp_s_sensor(id_sensor_nacelle1)
    
    Robotran.set_output_vector([mbs_data.sensor_passenger1.A[1], mbs_data.sensor_passenger1.A[2], mbs_data.sensor_passenger1.A[3]], 'qdd_sensor_passenger1')
    Robotran.set_output_vector([mbs_data.sensor_nacelle1.P[1], mbs_data.sensor_nacelle1.P[2], mbs_data.sensor_nacelle1.P[3]], 'q_sensor_nacelle1')
    
    return


def user_dirdyn_finish(mbs_data, mbs_dirdyn):
    """Run specific operations required by the user when direct dynamic analysis ends.

    Parameters
    ----------
    mbs_data : MBsysPy.MbsData
        The instance containing the multibody project.
    mbs_dirdyn : MBsysPy.MbsDirdyn
        The instance of the current direct dynamic process.

    Returns
    -------
    None.

    """

    # Example: The velocities are saved to a file, then the fields created in
    #          the MbsData instance during the function `user_dirdyn_init` are
    #          removed.
    #
    # import numpy as np # Should be done outside the function.
    #
    # np.savetxt("myfile.txt", mbs_data.my_sensor_v)
    # del mbs_data.my_sensor, mbs_data.my_sensor_v

    return
