using System;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using UnityEngine.Serialization;
using Quaternion = UnityEngine.Quaternion;
using Vector3 = UnityEngine.Vector3;

public class Agent : MonoBehaviour
{
    private List<Agent> _neighbors = new List<Agent>();
    private Rigidbody _rigidbody;

    [Header("Behaviour")]
    public GameObject target;
    [Header("Movement")]
    public float maxVelocity;
    public float minVelocity;
    public float maxRotation;
    public float cohesionFactor;
    public float alignmentFactor;
    public float separationFactor;
    [Header("Arrival")]
    public float arrivalSlowingDistance;

    public Vector3 GetVelocity()
    {
        return _rigidbody.velocity;
    }
    
    // Process an object entering the trigger area
    private void OnTriggerEnter(Collider other)
    {
        // try to get the agent script
        var agent = other.gameObject.GetComponent<Agent>();
        // if the other game object is an agent, add it to the neighbors list
        if (agent) _neighbors.Add(agent);
        // TODO: handle solid terrain getting in range
    }

    // Process an object leaving the trigger area
    private void OnTriggerExit(Collider other)
    {
        // try to get the agent script
        var agent = other.gameObject.GetComponent<Agent>();
        // if the other game object is an agent, add it to the neighbors list
        if (agent) _neighbors.Remove(agent);
        // TODO: handle solid terrain leaving the range range
    }

    // "Seek" Steering Behaviour
    private Vector3 Seek(Transform target)
    {
        //return Vector3.Normalize(target.position - transform.position) - _velocity;
        return target.position - transform.position - _rigidbody.velocity;
    }
    
    // "Arrival" Steering Behaviour
    private Vector3 Arrival(Transform target)
    {
        var targetPosition = target.position;
        var ownPosition = transform.position;
        // calculate target distance and desired velocity
        var targetDistance = Vector3.Distance(targetPosition, ownPosition);
        var desiredVelocity = maxVelocity * (targetDistance / arrivalSlowingDistance);
        // calculate arrival velocity
        return ((Mathf.Min(desiredVelocity, maxVelocity) / targetDistance) * (targetPosition - ownPosition)) - _rigidbody.velocity;
    }

    // "Cohesion" Steering Behaviour
    private Vector3 Cohesion()
    {
        // calculate the average position of all nearby agents and return a vector pointing there
        return _neighbors.Count > 0 ? new Vector3(
                                          _neighbors.Average(o => o.transform.position.x),
                                          _neighbors.Average(o => o.transform.position.y),
                                          _neighbors.Average(o => o.transform.position.z)) - transform.position : Vector3.zero;
    }
    
    // "Separation" Steering Behaviour
    private Vector3 Separation()
    {
        var separationTarget = Vector3.zero;
        var ownPosition = transform.position;
        // sum up repulsive forces of all neighbors
        for (int i = 0; i < _neighbors.Count; i++)
        {
            // calculate position of the neighbor
            var neighborPosition = _neighbors[i].transform.position;
            // calculate repulsive force of the neighbor
            if (Vector3.Distance(ownPosition, neighborPosition) != 0.0f)
            {
                separationTarget += Vector3.Normalize(ownPosition - neighborPosition) /
                                    Mathf.Pow(Vector3.Distance(ownPosition, neighborPosition), 2);
            }
        }
        // return a vector pointing at the desired position
        return separationTarget;
    }
    
    // "Alignment" Steering Behaviour
    private Vector3 Alignment()
    {
        // calculate average velocity of all neighbors (or zero if there are none)
        var neighborVelocity = _neighbors.Count > 0 ? new Vector3(
            _neighbors.Average(o => o.GetVelocity().x),
            _neighbors.Average(o => o.GetVelocity().y),
            _neighbors.Average(o => o.GetVelocity().z)) : Vector3.zero;
        // return a normalized vector representing the target velocity (or zero if there are no neighbors
        return _neighbors.Count == 0 ? neighborVelocity : Vector3.Normalize(neighborVelocity - _rigidbody.velocity);
    }

    // Start is called before the first frame update
    void Start()
    {
        // set _rigidbody variable to the rigid body of the game object this is attached to
        _rigidbody = GetComponent<Rigidbody>();
        // set riggidbody velocity to speed
        // TODO
    }

    // Update is called once per frame
    private void FixedUpdate()
    {
        // log vectors
        //Debug.Log(String.Format("[{0}] RB velocity: {1}, Arrival: {2}, Cohesion: {3}, Separation: {4}, Alignment: {5}",
        //    gameObject.name, _rigidbody.velocity,  Arrival(target.transform), Cohesion(), Separation(), Alignment()));
        // move
        //_rigidbody.AddForce(CalculateMovement());
        //_rigidbody.MovePosition(transform.position + CalculateMovement() * Time.deltaTime);
        //_rigidbody.velocity = CalculateMovement();
        _rigidbody.AddForce(CalculateMovement(), ForceMode.VelocityChange);
    }

    // Calculates a movement vector for this agent
    Vector3 CalculateMovement()
    {
        // TODO: since every steering behaviour loops over all neighbors they should be combined to increase performance
        // calculate velocity by combining the different steering behaviours
        // TODO: do we need to consider current rigidbody velocity?
        var velocity = Arrival(target.transform) + cohesionFactor * Cohesion() +  
                       separationFactor * Separation() +  alignmentFactor* Alignment();
        // limit velocity
        if (velocity.magnitude > maxVelocity) velocity = velocity.normalized * maxVelocity;
        if (velocity.magnitude < minVelocity) velocity = velocity.normalized * minVelocity;
        // limit rotation
        // TODO: do properly
        // velocity = Vector3.RotateTowards(velocity, velocity, maxRotation * Mathf.Deg2Rad, maxVelocity);
        return velocity;
    }
}
