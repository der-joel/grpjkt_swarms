using System.Collections.Generic;
using UnityEngine;
using Quaternion = UnityEngine.Quaternion;
using Vector3 = UnityEngine.Vector3;

[SelectionBase]
public class Agent : MonoBehaviour
{
    private List<Collider> _neighbors = new List<Collider>();
    private List<Collider> _obstacles = new List<Collider>();
    private Rigidbody _rigidbody;
    private SteeringBehaviours _steering;

    [Header("Movement")]
    public float maxAcceleration = 10;
    public float minAcceleration = 0;
    public float maxVelocity = 10;
    [Header("Ratio")]
    public float cohesionFactor = 1;
    public float alignmentFactor = 1;
    public float separationFactor = 1;
    public float obstacleAvoidanceFactor = 1;
    public float wanderFactor = 0.5f;
    public float arrivalFactor = 0;
    public float seekFactor = 0;
    public float fleeFactor = 0;
    public float hideFactor = 0;
    [Header("Seek")]
    public Transform seekTarget;
    public float maxSeekVelocity = 10;
    [Header("Arrival")]
    public Transform arrivalTarget;
    public float maxArrivalVelocity = 10;
    public float arrivalSlowingDistance = 40;
    [Header("Cohesion")]
    public float maxCohesionVelocity = 22;
    [Header("Separation")]
    public float maxSeparationVelocity = 20;
    [Header("Alignment")]
    public float maxAlignmentVelocity = 20;
    [Header("Wander")]
    public float wanderJitter = 20;
    public float wanderCircleRadius = 1.2f;
    public float wanderDistance = 2f;
    public float maxWanderVelocity = 10;
    [Header("Flee")]
    public Transform fleeTarget;
    public float maxFleeDistance = 20;
    public float maxFleeVelocity = 20;
    [Header("Hide")] 
    public Transform hideTarget;
    public float hideObstacleDistance = 10;
    public float maxHideVelocity = 20;
    [Header("Obstacle Avoidance")]
    public float maxObstacleAvoidanceVelocity = 25;

    private bool _isseekTargetNotNull;
    private bool _isarrivalTargetNotNull;
    private bool _isfleeTargetNotNull;
    private bool _ishideTargetNotNull;

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
        _ishideTargetNotNull = hideTarget != null;
        _isfleeTargetNotNull = fleeTarget != null;
        _isarrivalTargetNotNull = arrivalTarget != null;
        _isseekTargetNotNull = seekTarget != null;
        // set variables to the attached components
        _rigidbody = GetComponent<Rigidbody>();
        // calculate max sensor range (used for wall avoidance)
        var maxSensorRange = 0f;
        foreach (var sensor in GetComponents<Collider>())
        {
            // ignore non-trigger colliders
            if (!sensor.isTrigger) continue;
            // calculate max extend of bounds
            var boundsExtend = sensor.bounds.extents;
            var maxBoundsExtend = Mathf.Max(Mathf.Max(boundsExtend.x, boundsExtend.y), boundsExtend.z);
            var sensorRange = Vector3.Distance(transform.position, sensor.transform.position) + maxBoundsExtend;
            if (sensorRange > maxSensorRange) maxSensorRange = sensorRange;
        }
        // create steering behaviour class
        _steering = new SteeringBehaviours(
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
            maxObstacleAvoidanceVelocity,
            hideObstacleDistance,
            maxHideVelocity);
    }
    
    // Draw gizmos when this agent is selected in the scene view
    private void OnDrawGizmosSelected()
    {
        if (_steering == null) return;
        var currentPosition = transform.position;
        // calculate steering
        Vector3 seek = Vector3.zero, arrival = Vector3.zero, flee = Vector3.zero, hide = Vector3.zero;
        var alignment = alignmentFactor * _steering.Alignment(_neighbors);
        var cohesion = cohesionFactor * _steering.Cohesion(currentPosition, _neighbors);
        var separation  = separationFactor * _steering.Separation(currentPosition, _neighbors);
        var obstacleAvoidance = obstacleAvoidanceFactor * _steering.ObstacleAvoidance(currentPosition, _rigidbody.velocity, _obstacles);
        var wander = wanderFactor * _steering.Wander(currentPosition, transform.forward);
        if (_isarrivalTargetNotNull) arrival = arrivalFactor * _steering.Arrival(currentPosition, arrivalTarget.position);
        if (_isseekTargetNotNull) seek = seekFactor * _steering.Seek(currentPosition, seekTarget.position);
        if (_isfleeTargetNotNull) flee = fleeFactor * _steering.Flee(currentPosition, fleeTarget.position);
        if (_ishideTargetNotNull) hide = hideFactor * _steering.Hide(currentPosition, hideTarget.position, _obstacles);
        // calculate acceleration (combine steering behaviours and limit acceleration)
        Vector3 acceleration = alignment + cohesion + separation + obstacleAvoidance + wander + arrival + seek + flee + hide;
        if (acceleration.magnitude > maxAcceleration) acceleration = acceleration.normalized * maxAcceleration;
        if (acceleration.magnitude < minAcceleration) acceleration = acceleration.normalized * minAcceleration;
        // draw vector gizmos
        Gizmos.color = Color.red;
        Gizmos.DrawRay(currentPosition, cohesion);
        Gizmos.color = Color.green;
        Gizmos.DrawRay(currentPosition, separation);
        Gizmos.color = Color.blue;
        Gizmos.DrawRay(currentPosition, alignment);
        Gizmos.color = Color.white;
        Gizmos.DrawRay(currentPosition, obstacleAvoidance);
        Gizmos.color = Color.grey;
        Gizmos.DrawRay(currentPosition, flee);
        Gizmos.color = Color.black;
        Gizmos.DrawRay(currentPosition, hide);
        Gizmos.color = Color.yellow;
        Gizmos.DrawRay(currentPosition, wander);
        Gizmos.DrawRay(currentPosition, arrival);
        Gizmos.DrawRay(currentPosition, seek);
        Gizmos.color = Color.magenta;
        Gizmos.DrawRay(currentPosition, acceleration);
    }

    private Vector3 GetSteering(Vector3 currentPosition)
    {
        // calculate steering
        Vector3 seek = Vector3.zero, arrival = Vector3.zero, flee = Vector3.zero, hide = Vector3.zero;
        var alignment = alignmentFactor * _steering.Alignment(_neighbors);
        var cohesion = cohesionFactor * _steering.Cohesion(currentPosition, _neighbors);
        var separation  = separationFactor * _steering.Separation(currentPosition, _neighbors);
        var obstacleAvoidance = obstacleAvoidanceFactor * _steering.ObstacleAvoidance(currentPosition, _rigidbody.velocity, _obstacles);
        var wander = wanderFactor * _steering.Wander(currentPosition, transform.forward);
        if (_isarrivalTargetNotNull) arrival = arrivalFactor * _steering.Arrival(currentPosition, arrivalTarget.position);
        if (_isseekTargetNotNull) seek = seekFactor * _steering.Seek(currentPosition, seekTarget.position);
        if (_isfleeTargetNotNull) flee = fleeFactor * _steering.Flee(currentPosition, fleeTarget.position);
        if (_ishideTargetNotNull) hide = hideFactor * _steering.Hide(currentPosition, hideTarget.position, _obstacles);

        // calculate acceleration (combine steering behaviours)
        var acceleration = alignment + cohesion + separation + obstacleAvoidance + wander + arrival + seek + flee + hide;
        
        // limit acceleration
        if (acceleration.magnitude > maxAcceleration) acceleration = acceleration.normalized * maxAcceleration;
        if (acceleration.magnitude < minAcceleration) acceleration = acceleration.normalized * minAcceleration;

        return acceleration;
    }

    // FixedUpdate is called every time the physics engine updates
    private void FixedUpdate()
    {
        // calculate acceleration
        var acceleration = GetSteering(transform.position);
        // accelerate rigidbody
        _rigidbody.AddForce(acceleration, ForceMode.Acceleration);
        // limit velocity
        if (_rigidbody.velocity.magnitude > maxVelocity)
        {
            _rigidbody.velocity = _rigidbody.velocity.normalized * maxVelocity;
        }
        // turn agent towards the move direction
        if (_rigidbody.velocity != Vector3.zero) _rigidbody.rotation = Quaternion.LookRotation(Vector3.Normalize(_rigidbody.velocity));
    }
}
