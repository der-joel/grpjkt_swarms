using System;
using System.Collections.Generic;
using System.Linq;
using UnityEditor;
using UnityEngine;
using UnityEngine.Serialization;
using Quaternion = UnityEngine.Quaternion;
using Vector3 = UnityEngine.Vector3;


public class Agent : MonoBehaviour
{
    private List<Collider> _neighbors = new List<Collider>();
    private List<Collider> _obstacles = new List<Collider>();
    private Rigidbody _rigidbody;
    private SteeringBehaviours _steering;

    [Header("Behaviour")]
    public GameObject target;

    [Header("Movement")] 
    public float maxTurnSpeed;
    public float maxVelocity;
    public float minVelocity;
    public float maxRotation;
    public float cohesionFactor = 1.5f;
    public float alignmentFactor = 1f;
    public float separationFactor = 2f;
    public float arrivalFactor = 3;
    public float obstacleAvoidanceFactor = 5;
    public float wanderFactor = 1;

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
        _steering = GetComponent<SteeringBehaviours>();
    }

    // FixedUpdate is called every time the engine updates
    private void FixedUpdate()
    {
        // calculate steering
        var alignment = alignmentFactor * _steering.Alignment(_neighbors);
        var cohesion = cohesionFactor * _steering.Cohesion(_neighbors);
        var separation  = separationFactor * _steering.Separation(_neighbors);
        var arrival = arrivalFactor * _steering.Arrival(target.transform.position);
        var seek = arrivalFactor * _steering.Seek(target.transform.position);
        var obstacleAvoidance = obstacleAvoidanceFactor * _steering.ObstacleAvoidance(_obstacles);
        var wander = wanderFactor * _steering.Wander();

        // calculate acceleration
        Vector3 acceleration = alignment + cohesion + separation + obstacleAvoidance + wander;
        // limit acceleration
        if (acceleration.magnitude > maxVelocity) acceleration = acceleration.normalized * maxVelocity;
        if (acceleration.magnitude < minVelocity) acceleration = acceleration.normalized * minVelocity;
        
        // show debug information
        var currentPosition = transform.position;
        if (obstacleAvoidance != Vector3.zero) Debug.DrawLine(currentPosition, currentPosition + obstacleAvoidance.normalized, Color.cyan);
        if (wander != Vector3.zero) Debug.DrawLine(currentPosition, currentPosition + wander, Color.yellow);
        
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
