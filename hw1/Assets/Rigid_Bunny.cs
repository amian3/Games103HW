using UnityEngine;
using System.Collections;

public class Rigid_Bunny : MonoBehaviour
{
    bool launched = false;
    float dt = 0.015f;
    Vector3 v = new Vector3(0, 0, 0);   // velocity
    Vector3 w = new Vector3(0, 0, 0);   // angular velocity

    float mass;                                 // mass
    Matrix4x4 I_ref;                            // reference inertia

    float linear_decay = 0.999f;                // for velocity decay
    float angular_decay = 0.98f;
    float restitution = 0.5f;                 // for collision

    float muN = 0.5f;
    float muT = 0.2f;
    // Use this for initialization
    void Start()
    {
        Mesh mesh = GetComponent<MeshFilter>().mesh;
        Vector3[] vertices = mesh.vertices;

        float m = 1;
        mass = 0;
        for (int i = 0; i < vertices.Length; i++)
        {
            mass += m;
            float diag = m * vertices[i].sqrMagnitude;
            I_ref[0, 0] += diag;
            I_ref[1, 1] += diag;
            I_ref[2, 2] += diag;
            I_ref[0, 0] -= m * vertices[i][0] * vertices[i][0];
            I_ref[0, 1] -= m * vertices[i][0] * vertices[i][1];
            I_ref[0, 2] -= m * vertices[i][0] * vertices[i][2];
            I_ref[1, 0] -= m * vertices[i][1] * vertices[i][0];
            I_ref[1, 1] -= m * vertices[i][1] * vertices[i][1];
            I_ref[1, 2] -= m * vertices[i][1] * vertices[i][2];
            I_ref[2, 0] -= m * vertices[i][2] * vertices[i][0];
            I_ref[2, 1] -= m * vertices[i][2] * vertices[i][1];
            I_ref[2, 2] -= m * vertices[i][2] * vertices[i][2];
        }
        I_ref[3, 3] = 1;
    }

    Matrix4x4 Get_Cross_Matrix(Vector3 a)
    {
        //Get the cross product matrix of vector a
        Matrix4x4 A = Matrix4x4.zero;
        A[0, 0] = 0;
        A[0, 1] = -a[2];
        A[0, 2] = a[1];
        A[1, 0] = a[2];
        A[1, 1] = 0;
        A[1, 2] = -a[0];
        A[2, 0] = -a[1];
        A[2, 1] = a[0];
        A[2, 2] = 0;
        A[3, 3] = 1;
        return A;
    }

    // In this function, update v and w by the impulse due to the collision with
    //a plane <P, N>
    void Collision_Impulse(Vector3 P, Vector3 N)
    {
        Mesh mesh = GetComponent<MeshFilter>().mesh;
        Vector3[] vertices = mesh.vertices;
        Matrix4x4 R = Matrix4x4.Rotate(transform.rotation);

        Vector3 AveragePoint = new Vector3(0, 0, 0);
        int num = 0;
        for (int i = 0; i < vertices.Length; i++)
        {
            Vector3 xi = transform.position + R.MultiplyVector(vertices[i]);
            if (Vector3.Dot(xi - P, N) < 0)
            {
                Vector3 vvi = v + Vector3.Cross(w, R.MultiplyVector(vertices[i]));
                if (Vector3.Dot(vvi, N) < 0)
                {
                    num++;
                    AveragePoint += vertices[i];
                }
            }
        }
        if (num == 0) return;
        AveragePoint /= num;

        Vector3 vi = v + Vector3.Cross(w, R.MultiplyVector(AveragePoint));
        Vector3 vNi = Vector3.Dot(vi, N) * N;
        Vector3 vTi = vi - vNi;
        float alpha = Mathf.Max(1 - muT * (1 + muN) * Vector3.Magnitude(vNi) / Vector3.Magnitude(vTi), 0);
        Vector3 vnewNi = -muN * vNi;
        Vector3 vnewTi = alpha * vTi;
        Vector3 vnewi = vnewNi + vnewTi;

        Matrix4x4 IbyM = Matrix4x4.identity;
        for (int first = 0; first < 4; first++)
        {
            for (int second = 0; second < 4; second++)
            {
                IbyM[first, second] = 1 / mass * IbyM[first, second];

            }
        }
        Matrix4x4 minuspart = Get_Cross_Matrix(R.MultiplyVector(AveragePoint)) * Matrix4x4.Inverse(I_ref);
        minuspart = minuspart * Get_Cross_Matrix(R.MultiplyVector(AveragePoint));
        Matrix4x4 K = Matrix4x4.identity;
        for (int first = 0; first < 4; first++)
        {
            for (int second = 0; second < 4; second++)
            {
                K[first, second] = IbyM[first, second] - minuspart[first, second];
            }
        }
        Vector3 J = K.inverse.MultiplyVector(vnewi - vi);
        v = v + 1 / mass * J;
        w = w + Matrix4x4.Inverse(I_ref).MultiplyVector(Vector3.Cross(R.MultiplyVector(AveragePoint), J));
    }

    // Update is called once per frame
    void Update()
    {
        //Game Control
        if (Input.GetKey("r"))
        {
            transform.position = new Vector3(0, 0.6f, 0);
            restitution = 0.5f;
            launched = false;
        }
        if (Input.GetKey("l"))
        {
            v = new Vector3(5, 2, 0);
            launched = true;
        }
        if (launched)
        {
            // Part I: Update velocities
            Vector3 gforce = new Vector3(0, -9.8f, 0);
            v = v + gforce * dt;
            v = linear_decay * v;
            w = angular_decay * w;

            Debug.Log(v);
            // Part II: Collision Impulse
            Collision_Impulse(new Vector3(0, 0.01f, 0), new Vector3(0, 1, 0));
            Collision_Impulse(new Vector3(2, 0, 0), new Vector3(-1, 0, 0));

            // Part III: Update position & orientation
            //Update linear status
            Vector3 x = transform.position;
            x = x + dt * v;
            //Update angular status
            Quaternion q = transform.rotation;

            Quaternion diffq = new Quaternion(w.x * dt / 2, w.y * dt / 2, w.z * dt / 2, 0);
            diffq = diffq * q;
            q.x = q.x + diffq.x;
            q.y = q.y + diffq.y;
            q.z = q.z + diffq.z;
            q.w = q.w + diffq.w;
            // Part IV: Assign to the object
            transform.position = x;
            transform.rotation = q;
        }

    }
}
