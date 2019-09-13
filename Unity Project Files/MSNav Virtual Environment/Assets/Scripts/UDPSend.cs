// Unity Environment for Microsnapshot Navigation
// Author: Tristan Baumann

using UnityEngine;
using System.Collections;
using System;
using System.Text;
using System.Net;
using System.Net.Sockets;
using System.Threading;


[RequireComponent(typeof(CharacterController))]
public class UDPSend : MonoBehaviour
{
    // Variable declaration ===============================================

    // Editor fields
    [SerializeField] private bool m_IsWalking;
    [SerializeField] private float m_WalkSpeed;
    [SerializeField] private float m_RunSpeed;
    [SerializeField] private float m_StickToGroundForce;
    [SerializeField] private float m_GravityMultiplier;

    // Agent (Camera and collision)
    private Camera m_Camera;
    private Camera CamR;
    private float m_YRotation;
    private Vector2 m_Input;
    private Vector3 m_MoveDir = Vector3.zero;
    private CharacterController m_CharacterController;
    private CollisionFlags m_CollisionFlags;
    private bool m_PreviouslyGrounded;
    private Vector3 m_OriginalCameraPosition;
    private Vector3 m_OriginalGameControllerPosition;
    private GameObject cylinder;

    // UDP connections (defined in init())
    private string IP;
    private int port;
    private int port_rec;

    IPEndPoint remoteEndPoint;
    UdpClient client;

    Thread receiveThread;
    UdpClient client_rec;

    // Data transfer
    private float recx = 0;
    private float recz = 0;
    private float recR = 0;
    private float movex = 0;
    private float movez = 0;
    private float recx_old = 0;
    private float recz_old = 0;
    private bool sendImage = true;

    string text;

    private float goalx = 0;
    private float goalz = 0;

    // Rastering
    //private Vector3 rasterPoint;
    //private float time = 0.5f;
    //private bool rasterStart = true;
    //private float rasterBorderX = 188.0f;
    //private float rasterStartX = -155;

    // Increases movement speed on receiving 
    // movement instructions to offset slower movement.
    private float velocityMultiplier = 2.0f;

    // Random walk
    private Vector3 randomMove;
    private double lastInterval = 0;

    private void Awake()
    {
        // Frame rate is capped at 30fps
        Application.targetFrameRate = 30;
    }

    public void Start()
    {
        init();
    }

    // Initializes the program
    // Starts UPD data transmission threads and coroutines
    public void init()
    {
        // Agent initialization
        m_CharacterController = GetComponent<CharacterController>();
        m_Camera = Camera.main;
        m_OriginalCameraPosition = m_Camera.transform.localPosition;
        m_OriginalGameControllerPosition = m_CharacterController.transform.position;
        cylinder = GameObject.Find("Cylinder_actor");

        // UDP sending
        IP = "127.0.0.1";
        port = 55551;
        remoteEndPoint = new IPEndPoint(IPAddress.Parse(IP), port);
        client = new UdpClient();

        // Starts UDP sending coroutine
        StartCoroutine(sendCoroutine());

        // UDP receiving
        port_rec = 55552;
        // starts UDP receiving thread
        receiveThread = new Thread(
            new ThreadStart(ReceiveData));
        receiveThread.IsBackground = true;
        receiveThread.Start();
        
        // set timer for random walk
        lastInterval = Time.realtimeSinceStartup;

        // initial random walk direction
        randomMove = transform.forward;

        // rasterization
        //rasterPoint.x = -115;
        //rasterPoint.y = -6;
        //rasterPoint.z = 60;
    }

    // UDP data receiving function 
    // Runs in a separate thread.
    private void ReceiveData()
    {
        client_rec = new UdpClient(port_rec);
        while (true)
        {
            // Receiving data
            IPEndPoint anyIP = new IPEndPoint(IPAddress.Any, 0);
            byte[] data = client_rec.Receive(ref anyIP);
            text = Encoding.UTF8.GetString(data); // decode data into string

            // Parse string into three float values separated by p and q
            int separator = text.IndexOf("p");
            int separator2 = text.IndexOf("q");
            recx = float.Parse(text.Substring(0, separator - 1));
            recz = float.Parse(text.Substring(separator + 1, separator2 - separator - 2));
            recR = float.Parse(text.Substring(separator2 + 1, text.Length - separator2 - 2));
        }
    }

    // UPD data sending coroutine
    private IEnumerator sendCoroutine()
    {
        while (true)
        {
            yield return new WaitForEndOfFrame();
            try
            {
                // Screenshot
                Texture2D screenImage = new Texture2D(1280, 240);
                screenImage.ReadPixels(new Rect(0, 480, 1280, 240), 0, 0);

                // Encode screenshot as a compressed JPEG to fit into one buffer
                byte[] imgdata = screenImage.EncodeToJPG(75);
                UnityEngine.Object.Destroy(screenImage);

                // Get positional and rotational data
                Vector3 positions = Camera.main.gameObject.transform.position;
                byte[] position = new byte[16];
                System.BitConverter.GetBytes(positions.x).CopyTo(position, 0);
                System.BitConverter.GetBytes(positions.y).CopyTo(position, 4);
                System.BitConverter.GetBytes(positions.z).CopyTo(position, 8);
                System.BitConverter.GetBytes(Camera.main.gameObject.transform.eulerAngles.y).CopyTo(position, 12);

                // append data
                byte[] data1;
                data1 = new byte[imgdata.Length + 16];
                System.Buffer.BlockCopy(imgdata, 0, data1, 0, imgdata.Length);
                System.Buffer.BlockCopy(position, 0, data1, imgdata.Length, 16);

                // send data
                if (sendImage)
                    client.Send(data1, data1.Length, remoteEndPoint);
            }
            catch (System.Exception err)
            {
                print(err.ToString());
            }
        }
    }

    // Random walk function
    // Assigns new semirandom direction to movement
    private void RandomWalk()
    {
        // Reflect movement off of walls if agent hits a wall
        RaycastHit hitInfo;
        Physics.SphereCast(transform.position, m_CharacterController.radius, randomMove, out hitInfo,
                           0.5f, Physics.AllLayers, QueryTriggerInteraction.Ignore);

        Vector3 reflectedMove = Vector3.Reflect(randomMove, hitInfo.normal);
        if (reflectedMove[0] == 0 && reflectedMove[2] == 0)
        {
            randomMove = transform.forward;
        }
        else
        {
            randomMove[0] = reflectedMove[0];
            randomMove[2] = reflectedMove[2];
        }

        // Randomly change movement direction by up to 20° in either 
        // direction every .5 seconds
        if (Time.realtimeSinceStartup - lastInterval > 0.5f)
        {
            float mod = UnityEngine.Random.Range(-20f, 20f);
            randomMove = Quaternion.Euler(0, mod, 0) * randomMove;
            lastInterval = Time.realtimeSinceStartup;
        }

        randomMove.y = 0; // Remove evtl. effects of reflection on the y-axis
        randomMove.Normalize();

        // assign movements 
        movex = randomMove.x;
        movez = randomMove.z;
    }

    // Returns agent (cameras and character controller) to its starting position
    // And stop movement
    private void teleportToStart()
    {
        m_CharacterController.transform.position = m_OriginalGameControllerPosition;
        goalx = m_CharacterController.transform.position.x;
        goalz = m_CharacterController.transform.position.z;
        recx = 0;
        recz = 0;
        recx_old = recx;
        recz_old = recz;
        movex = 0;
        movez = 0;
    }

    // Performs steps in a raster
    //private void placeFieldRastering()
    //{
    //    if (rasterStart)
    //    {
    //        rasterStart = false;
    //        rasterStartX = ((int)m_CharacterController.transform.position.x) - 22;
    //        rasterPoint.x = rasterStartX;
    //        rasterPoint.z = ((int)m_CharacterController.transform.position.z) + 12;
    //        rasterBorderX = rasterPoint.x + 44;
    //        //  rasterPoint.y = ((int)m_CharacterController.transform.position.z) + 5;
    //        m_GravityMultiplier = 30;
    //    }

    //    if (time > 0)
    //    {
    //        time -= Time.deltaTime;
    //    }
    //    else
    //    {
    //        time = 0.3f;
    //        m_CharacterController.transform.position = rasterPoint;
    //        rasterPoint.x += 0.50f;
    //        if (rasterPoint.x > rasterBorderX)
    //        {
    //            rasterPoint.x = rasterStartX;
    //            rasterPoint.z -= 0.50f;
    //        }
    //    }
    //}

    // Frake rate indepedentent update function
    // mainly for physics-related calculations. 
    // Is called 50? times per second
    private void FixedUpdate()
    {
        float speed;
        GetInput(out speed); // speed corresponds to received movement vector length

        // the third entry of the received array contains orders
        // They are:
        if (recR == 5) // starts random walk
        {
            RandomWalk();
        }
        else if (recR == 6) // teleports to the starting position
        {
            teleportToStart();
        }
        //else if (recR == 7) //place field mapping
        //{
        //    if (totalMovement.magnitude < 0.01)
        //    {
        //        sendImage = true;
        //    }
        //    else
        //    {
        //        sendImage = false;
        //    }
        //    placeFieldRastering();
        //}
        else // default (e.g. 0) for stopping or movement
        {
            speed = 1.5f * speed;

            if (recx == 0 && recz == 0) // if no movement instruction is received, stop
            {
                movex = 0;
                movez = 0;
            }
            else
            {
                if ((recx_old != recx) || (recz_old != recz)) // set "goals" for the next movement step
                {
                    recx_old = recx;
                    recz_old = recz;
                    goalx = Camera.main.gameObject.transform.position.x + recx*velocityMultiplier;
                    goalz = Camera.main.gameObject.transform.position.z + recz*velocityMultiplier;
                }

                movex = goalx - Camera.main.gameObject.transform.position.x;
                movez = goalz - Camera.main.gameObject.transform.position.z;
            }
        }

        // Calculate desired agent movement direction
        Vector3 desiredMove = transform.forward * m_Input.y + transform.right * m_Input.x;
        desiredMove.x += movex;
        desiredMove.z += movez;
        // Add physics such as obstacles or the ground
        RaycastHit hitInfo;
        Physics.SphereCast(transform.position, m_CharacterController.radius, Vector3.down, out hitInfo,
                           m_CharacterController.height / 2f, Physics.AllLayers, QueryTriggerInteraction.Ignore);
        desiredMove = Vector3.ProjectOnPlane(desiredMove, hitInfo.normal);//.normalized;
        if (desiredMove.magnitude > 1)
            desiredMove.Normalize();
        // multiply with desired speed
        m_MoveDir.x = desiredMove.x * speed;
        m_MoveDir.z = desiredMove.z * speed;
        // gravity
        if (m_CharacterController.isGrounded)
        {
            m_MoveDir.y = -m_StickToGroundForce;
        }
        else
        {
            m_MoveDir += Physics.gravity * m_GravityMultiplier * Time.fixedDeltaTime;
        }
        // perform movement
        m_CollisionFlags = m_CharacterController.Move(m_MoveDir * Time.fixedDeltaTime);
        UpdateCameraPosition(speed);
        cylinder.transform.LookAt(cylinder.transform.position + desiredMove);
    }

    // moves the panorama camera(s)
    private void UpdateCameraPosition(float speed)
    {
        Vector3 newCameraPosition;

        if (m_CharacterController.velocity.magnitude > 0 && m_CharacterController.isGrounded)
        {
            newCameraPosition = m_Camera.transform.localPosition;
            newCameraPosition.y = m_Camera.transform.localPosition.y;
        }
        else
        {
            newCameraPosition = m_Camera.transform.localPosition;
            newCameraPosition.y = m_OriginalCameraPosition.y;
        }
        m_Camera.transform.localPosition = newCameraPosition;
    }

    // Key input function
    // The agent may be controlled by the user
    private void GetInput(out float speed)
    {
        // Read input
        float horizontal = Input.GetAxis("Horizontal");
        float vertical = Input.GetAxis("Vertical");

        // The shift key will make the agent move faster
        m_IsWalking = !Input.GetKey(KeyCode.LeftShift);
        speed = m_IsWalking ? m_WalkSpeed : m_RunSpeed;

        m_Input = new Vector2(horizontal, vertical);

        // normalize input if it exceeds 1 in combined length:
        if (m_Input.sqrMagnitude > 1)
        {
            m_Input.Normalize();
        }

        // Holding down the r key will let the agent random walk
        if (Input.GetKey(KeyCode.R))
        {
            RandomWalk();
        }

    }

    // Pressing q or e will rotate the camera
    // Currently unused
    private void RotateView()
    {
        float rotRight = Input.GetAxis("e");
        float rotLeft = Input.GetAxis("q");
        transform.localRotation *= Quaternion.Euler(0, 3 * rotRight - 3 * rotLeft /*+ 0.3f * moveR*/, 0);

    }

    // Physics collision detection
    private void OnControllerColliderHit(ControllerColliderHit hit)
    {
        Rigidbody body = hit.collider.attachedRigidbody;
        //dont move the rigidbody if the character is on top of it
        if (m_CollisionFlags == CollisionFlags.Below)
        {
            return;
        }

        if (body == null || body.isKinematic)
        {
            return;
        }
        body.AddForceAtPosition(m_CharacterController.velocity * 0.1f, hit.point, ForceMode.Impulse);
    }

    private void OnApplicationQuit()
    {
        receiveThread.Abort();
        if (client != null) client.Close();
        if (client_rec != null) client_rec.Close();
    }

}
