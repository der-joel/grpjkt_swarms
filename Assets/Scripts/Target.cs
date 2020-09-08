using UnityEngine;

public class Target : MonoBehaviour
{
    private CharacterController _controller;

    [Header("Target Properties")]
    public float speed;
    
    // Start is called before the first frame update
    void Start()
    {
        // set up character controller
        _controller = GetComponent<CharacterController>();
    }

    // Update is called once per frame
    void Update()
    {
        // movement
        //var move = transform.forward * Input.GetAxis("Vertical");
        //move += transform.right * Input.GetAxis("Horizontal");
        //_controller.Move(move * (speed * Time.deltaTime));
    }
}
