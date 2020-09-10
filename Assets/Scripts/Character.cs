using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Character : MonoBehaviour
{
    private float _turnVelocity;
    private Vector2 _screenCenter;

    [Header("Character Properties")]
    public float speed = 10;
    public float lookRotateSpeed = 90f;
    public float rollSpeed = 45;

    // Start is called before the first frame update
    void Start()
    {
        // set screen center
        _screenCenter.x = Screen.width * .5f;
        _screenCenter.y = Screen.height * .5f;
    }

    // Update is called once per frame
    void Update()
    {
        // calculate rotation from mouse position
        var lookInput = Input.mousePosition;
        var mouseDistance = new Vector2(
            (lookInput.x - _screenCenter.x) / _screenCenter.y,
            (lookInput.y - _screenCenter.y) / _screenCenter.y);
        mouseDistance = Vector2.ClampMagnitude(mouseDistance, 1f);
        // calculate roll from horizontal axis
        var roll = -Input.GetAxis("Horizontal");
        // rotate character
        transform.Rotate(
            -mouseDistance.y * lookRotateSpeed * Time.deltaTime, 
            mouseDistance.x * lookRotateSpeed * Time.deltaTime,
            roll * rollSpeed * Time.deltaTime,
            Space.Self);
        // move forward/backward
        var vertical = Input.GetAxis("Vertical");
        transform.position += transform.forward * (vertical * speed * Time.deltaTime);
    }
}
