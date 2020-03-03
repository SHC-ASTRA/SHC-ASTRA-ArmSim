using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class TargetDriver : MonoBehaviour
{

    public float translateSpeed;
    public float rotateSpeed;

    // Start is called before the first frame update
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        Vector3 translate = new Vector3();
        if(Input.GetKey(KeyCode.W))
        {
            translate += Vector3.forward;
        }
        if (Input.GetKey(KeyCode.S))
        {
            translate += Vector3.back;
        }
        if (Input.GetKey(KeyCode.A))
        {
            translate += Vector3.left;
        }
        if (Input.GetKey(KeyCode.D))
        {
            translate += Vector3.right;
        }
        if (Input.GetKey(KeyCode.Q))
        {
            translate += Vector3.down;
        }
        if (Input.GetKey(KeyCode.E))
        {
            translate += Vector3.up;
        }

        Vector3 rotate = new Vector3();

        if (Input.GetKey(KeyCode.J))
        {
            rotate += Vector3.forward;
        }
        if (Input.GetKey(KeyCode.L))
        {
            rotate += Vector3.back;
        }
        if (Input.GetKey(KeyCode.I))
        {
            rotate += Vector3.left;
        }
        if (Input.GetKey(KeyCode.K))
        {
            rotate += Vector3.right;
        }
        if (Input.GetKey(KeyCode.U))
        {
            rotate += Vector3.down;
        }
        if (Input.GetKey(KeyCode.O))
        {
            rotate += Vector3.up;
        }

        transform.Translate(translate * translateSpeed * Time.deltaTime, Space.World);
        transform.Rotate(rotate * rotateSpeed * Time.deltaTime, Space.Self);
    }
}
