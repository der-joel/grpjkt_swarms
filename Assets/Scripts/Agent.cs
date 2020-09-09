using System;
using System.Collections.Generic;
using System.Linq;
using UnityEditor;
using UnityEngine;
using UnityEngine.Serialization;
using Quaternion = UnityEngine.Quaternion;
using Vector3 = UnityEngine.Vector3;

[SelectionBase]
public class Agent : MonoBehaviour
{
    private List<Collider> _neighbors = new List<Collider>();
    private List<Collider> _obstacles = new List<Collider>();
    private Rigidbody _rigidbody;
    private SteeringBehaviours _steering;

    [Header("Behaviour Settings")]
    [Header("Ratio")]
    public float cohesionFactor = 1;
    public float alignmentFactor = 1;
    public float separationFactor = 1;
    public float obstacleAvoidanceFactor = 2;
    public float wanderFactor = 1;
    public float arrivalFactor = 0;
    public float seekFactor = 0;
    public float fleeFactor = 0;
    [Header("Seek")]
    public Transform seekTarget;
    public float maxSeekVelocity = 3.5f;
    [Header("Arrival")]
    public Transform arrivalTarget;
    public float maxArrivalVelocity = 3.5f;
    public float arrivalSlowingDistance = 3.5f;
    [Header("Cohesion")]
    public float maxCohesionVelocity = 3.5f;
    [Header("Separation")]
    public float maxSeparationVelocity = 3.5f;
    [Header("Alignment")]
    public float maxAlignmentVelocity = 3.5f;
    [Header("Wander")]
    public float wanderJitter = 20;
    public float wanderCircleRadius = 1.2f;
    public float wanderDistance = 2f;
    public float maxWanderVelocity = 3.5f;
    [Header("Flee")]
    public Transform fleeTarget;
    public float maxFleeDistance = 20;
    public float maxFleeVelocity = 3.5f;
    [Header("Obstacle Avoidance")]
    public float maxObstacleAvoidanceVelocity = 3.5f;

    [Header("Movement")] 
    public float maxTurnSpeed;
    public float maxVelocity;
    public float minVelocity;

    // Process an object entering the trigger area
    private void OnTriggerEnter(Collider other)
    {
        // if the other game object is a swarm agent, add it to the neighbors list
        if (other.GetComponent<Agent>()) _neighbors.Add(other);
        // if not add it to the obstacle list (if its not a children of a swarm agent)
        else if (!other.GetComponentInParent<Agent>()) _obstacles.Add(other);
    }

    // Process an object leaving the trigger area
    private void OnTriggerExit(Collider other)
    {
        // if the other game object is a swarm agent, remove it from the neighbors list
        if (other.GetComponent<Agent>()) _neighbors.Remove(other);
        // if not remove it from the obstacle list (if its not a children of a swarm agent)
        else if (!other.GetComponentInParent<Agent>()) _obstacles.Remove(other);
    }

    // Start is called before the first frame update
    void Start()
    {
        // set variables to the attached components
        _rigidbody = GetComponent<Rigidbody>();
        // TODO: calculate max sensor range
        var maxSensorRange = 10;
        // create steering behaviour class
        _steering = new SteeringBehaviours(_rigidbody,
            maxSensorRange,
            maxSeekVelocity,
            maxArrivalVelocity,
            arrivalSlowingDistance,
            maxCohesionVelocity,
            maxSeparationVelocity,
            maxAlignmentVelocity,
            wanderJitter,
            wanderCircleRadius,
            wanderDistance,
            maxWanderVelocity,
            maxFleeDistance,
            maxFleeVelocity,
            maxObstacleAvoidanceVelocity);
    }

    // FixedUpdate is called every time the physics engine updates
    private void FixedUpdate()
    {
        var currentPosition = transform.position;
        
        // calculate steering
        var alignment = alignmentFactor * _steering.Alignment(_neighbors);
        var cohesion = cohesionFactor * _steering.Cohesion(currentPosition, _neighbors);
        var separation  = separationFactor * _steering.Separation(currentPosition, _neighbors);
        var obstacleAvoidance = obstacleAvoidanceFactor * _steering.ObstacleAvoidance(currentPosition, _obstacles);
        var wander = wanderFactor * _steering.Wander(currentPosition, transform.forward);
        var arrival = arrivalFactor * _steering.Arrival(currentPosition, arrivalTarget.position);
        var seek = seekFactor * _steering.Seek(currentPosition, seekTarget.position);
        var flee = fleeFactor * _steering.Flee(currentPosition, fleeTarget.position);

        // calculate acceleration (combine steering behaviours)
        Vector3 acceleration = alignment + cohesion + separation + obstacleAvoidance + wander + arrival + seek + flee;
        
        // limit acceleration
        if (acceleration.magnitude > maxVelocity) acceleration = acceleration.normalized * maxVelocity;
        if (acceleration.magnitude < minVelocity) acceleration = acceleration.normalized * minVelocity;
        
        // show debug information
        //if (obstacleAvoidance != Vector3.zero) Debug.DrawLine(currentPosition, currentPosition + obstacleAvoidance, Color.blue);
        //if (wander != Vector3.zero) Debug.DrawLine(currentPosition, currentPosition + wander, Color.yellow);
        
        if (false)
        {
            // draw debug gizmos
            if (alignment != Vector3.zero) Debug.DrawLine(currentPosition, alignment.normalized, Color.green);
            if (cohesion != Vector3.zero) Debug.DrawLine(currentPosition, cohesion.normalized, Color.blue);
            if (separation != Vector3.zero) Debug.DrawLine(currentPosition, separation.normalized, Color.black);
            //if (seek != Vector3.zero) Debug.DrawLine(currentPosition, seek.normalized, Color.yellow);
            if (obstacleAvoidance != Vector3.zero) Debug.DrawLine(currentPosition, obstacleAvoidance, Color.cyan);
            if (wander != Vector3.zero)Debug.DrawLine(currentPosition, currentPosition + wander, Color.yellow);
            Debug.DrawLine(currentPosition, currentPosition + acceleration, Color.magenta);
            // Log forces
            Debug.Log(
                $"Alignment: {alignment}\nCohesion: {cohesion}\nSeparation: {separation}\nobstacleAvoidance: {obstacleAvoidance}\nWander: {wander}");
        }

        // TODO: how to apply acceleration to the rigidbody?
        // accelerate agent
        _rigidbody.velocity += acceleration * Time.deltaTime;
        if (_rigidbody.velocity.magnitude > maxVelocity)
        {
            _rigidbody.velocity = _rigidbody.velocity.normalized * maxVelocity;
        }
        // _rigidbody.AddForce(acceleration, ForceMode.Force);
        // _rigidbody.MovePosition(transform.position + acceleration * Time.deltaTime);
        
        // turn agent towards the move direction
        _rigidbody.rotation = Quaternion.LookRotation(Vector3.Normalize(_rigidbody.velocity));
    }

    public void FaceTowardsMovement()
    {
        // TODO: test if this is useful to turn towards current movement direction
        
        // calculate look direction
        var direction = Vector3.Normalize(_rigidbody.velocity);

        // If we have a non-zero direction then look towards that direciton otherwise do nothing
        if (direction.sqrMagnitude > 0.001f)
        {
            /* Mulitply by -1 because counter clockwise on the y-axis is in the negative direction */
            float toRotation = -1 * (Mathf.Atan2(direction.z, direction.x) * Mathf.Rad2Deg);
            float rotation = Mathf.LerpAngle(_rigidbody.rotation.eulerAngles.y, toRotation, Time.deltaTime * maxTurnSpeed);
            _rigidbody.rotation = Quaternion.Euler(0, rotation, 0);
        }
    }
}
