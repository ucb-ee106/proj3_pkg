# !/usr/bin/env python -W ignore::DeprecationWarning
"""
Grasp Metrics for C106B Grasp Planning Lab
Author: Chris Correa
"""
import numpy as np
from utils import vec, adj, look_at_general, find_grasp_vertices, normal_at_point
from casadi import Opti, sin, cos, tan, vertcat, mtimes, sumsqr, sum1

# Can edit to make grasp point selection more/less restrictive
MAX_GRIPPER_DIST = .075
MIN_GRIPPER_DIST = .03

def compute_force_closure(vertices, normals, num_facets, mu, gamma, object_mass, mesh):
    """
    Compute the force closure of some object at contacts, with normal vectors 
    stored in normals. You can use the line method described in the project document.
    If you do, you will not need num_facets. This is the most basic (and probably least useful)
    grasp metric.

    Parameters
    ----------
    vertices : 2x3 :obj:`numpy.ndarray`
        obj mesh vertices on which the fingers will be placed
    normals : 2x3 :obj:`numpy.ndarray`
        obj mesh normals at the contact points
    num_facets : int
        number of vectors to use to approximate the friction cone.  these vectors 
        will be along the friction cone boundary
    mu : float 
        coefficient of friction
    gamma : float
        torsional friction coefficient
    object_mass : float
        mass of the object
    mesh : :obj:`Trimesh`
        mesh object

    Returns
    -------
    float : 1 or 0 if the grasp is/isn't force closure for the object
    """
    raise NotImplementedError

def get_grasp_map(vertices, normals, num_facets, mu, gamma):
    """ 
    Defined in the book on page 219. Compute the grasp map given the contact
    points and their surface normals

    Parameters
    ----------
    vertices : 2x3 :obj:`numpy.ndarray`
        obj mesh vertices on which the fingers will be placed
    normals : 2x3 :obj:`numpy.ndarray`
        obj mesh normals at the contact points
    num_facets : int
        number of vectors to use to approximate the friction cone.  these vectors 
        will be along the friction cone boundary
    mu : float 
        coefficient of friction
    gamma : float
        torsional friction coefficient

    Returns
    -------
    6x8 :obj:`numpy.ndarray` : grasp map
    """
    raise NotImplementedError

def find_contact_forces(vertices, normals, num_facets, mu, gamma, desired_wrench):
    """
    Compute that contact forces needed to produce the desired wrench

    Parameters
    ----------
    vertices : 2x3 :obj:`numpy.ndarray`
        obj mesh vertices on which the fingers will be placed
    normals : 2x3 :obj:`numpy.ndarray`
        obj mesh normals at the contact points
    num_facets : int
        number of vectors to use to approximate the friction cone.  these vectors 
        will be along the friction cone boundary
    mu : float 
        coefficient of friction
    gamma : float
        torsional friction coefficient
    desired_wrench : 6x :obj:`numpy.ndarray` potential wrench to be produced

    Returns
    -------
    bool: whether contact forces can produce the desired_wrench on the object
    """
    raise NotImplementedError

def compute_gravity_resistance(vertices, normals, num_facets, mu, gamma, object_mass, mesh):
    """
    Gravity produces some wrench on your object. Computes how much normal force is required
    to resist the wrench produced by gravity.

    Parameters
    ----------
    vertices : 2x3 :obj:`numpy.ndarray`
        obj mesh vertices on which the fingers will be placed
    normals : 2x3 :obj:`numpy.ndarray`
        obj mesh normals at the contact points
    num_facets : int
        number of vectors to use to approximate the friction cone.  these vectors 
        will be along the friction cone boundary
    mu : float 
        coefficient of friction
    gamma : float
        torsional friction coefficient
    object_mass : float
        mass of the object
    mesh : :obj:`Trimesh`
        mesh object

    Returns
    -------
    float: quality of the grasp
    """
    raise NotImplementedError

def compute_robust_force_closure(vertices, normals, num_facets, mu, gamma, object_mass, mesh):
    """
    Should return a score for the grasp according to the robust force closure metric.

    Parameters
    ----------
    vertices : 2x3 :obj:`numpy.ndarray`
        obj mesh vertices on which the fingers will be placed
    normals : 2x3 :obj:`numpy.ndarray`
        obj mesh normals at the contact points
    num_facets : int
        number of vectors to use to approximate the friction cone.  these vectors 
        will be along the friction cone boundary
    mu : float 
        coefficient of friction
    gamma : float
        torsional friction coefficient
    object_mass : float
        mass of the object
    mesh : :obj:`Trimesh`
        mesh object

    Returns
    -------
    float: quality of the grasp
    """
    raise NotImplementedError

def compute_ferrari_canny(vertices, normals, num_facets, mu, gamma, object_mass, mesh):
    """
    Should return a score for the grasp according to the Ferrari Canny metric.
    Use your favourite python convex optimization package. We suggest casadi.

    Parameters
    ----------
    vertices : 2x3 :obj:`numpy.ndarray`
        obj mesh vertices on which the fingers will be placed
    normals : 2x3 :obj:`numpy.ndarray`
        obj mesh normals at the contact points
    num_facets : int
        number of vectors to use to approximate the friction cone.  these vectors 
        will be along the friction cone boundary
    mu : float 
        coefficient of friction
    gamma : float
        torsional friction coefficient
    object_mass : float
        mass of the object
    mesh : :obj:`Trimesh`
        mesh object

    Returns
    -------
    float: quality of the grasp
    """
    raise NotImplementedError
