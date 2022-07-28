using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using System.IO;

public class FVM : MonoBehaviour
{
    float dt = 0.003f;
    float mass = 1;
    float stiffness_0 = 20000.0f;
    float stiffness_1 = 5000.0f;
    float damp = 0.999f;
    float muN = 0.5f;
    float muT = 0.2f;
    int[] Tet;
    int tet_number;         //The number of tetrahedra

    Vector3[] Force;
    Vector3[] V;
    Vector3[] X;
    int number;             //The number of vertices

    Matrix4x4[] inv_Dm;

    //For Laplacian smoothing.
    Vector3[] V_sum;
    int[] V_num;

    bool[][] graph;

    int[][] neighbors;

    Vector3[] P = { new Vector3(0.0f, -3.0f, 0.0f)};
    Vector3[] N = { new Vector3(0.0f, 1.0f, 0.0f)};

    SVD svd = new SVD();

    // Start is called before the first frame update
    void Start()
    {
        // FILO IO: Read the house model from files.
        // The model is from Jonathan Schewchuk's Stellar lib.
        {
            string fileContent = File.ReadAllText("Assets/house2.ele");
            string[] Strings = fileContent.Split(new char[] { ' ', '\t', '\r', '\n' }, StringSplitOptions.RemoveEmptyEntries);

            tet_number = int.Parse(Strings[0]);
            Tet = new int[tet_number * 4];

            for (int tet = 0; tet < tet_number; tet++)
            {
                Tet[tet * 4 + 0] = int.Parse(Strings[tet * 5 + 4]) - 1;
                Tet[tet * 4 + 1] = int.Parse(Strings[tet * 5 + 5]) - 1;
                Tet[tet * 4 + 2] = int.Parse(Strings[tet * 5 + 6]) - 1;
                Tet[tet * 4 + 3] = int.Parse(Strings[tet * 5 + 7]) - 1;
            }
        }
        {
            string fileContent = File.ReadAllText("Assets/house2.node");
            string[] Strings = fileContent.Split(new char[] { ' ', '\t', '\r', '\n' }, StringSplitOptions.RemoveEmptyEntries);
            number = int.Parse(Strings[0]);
            X = new Vector3[number];
            for (int i = 0; i < number; i++)
            {
                X[i].x = float.Parse(Strings[i * 5 + 5]) * 0.4f;
                X[i].y = float.Parse(Strings[i * 5 + 6]) * 0.4f;
                X[i].z = float.Parse(Strings[i * 5 + 7]) * 0.4f;
            }
            //Centralize the model.
            Vector3 center = Vector3.zero;
            for (int i = 0; i < number; i++) center += X[i];
            center = center / number;
            for (int i = 0; i < number; i++)
            {
                X[i] -= center;
                float temp = X[i].y;
                X[i].y = X[i].z;
                X[i].z = temp;
            }
        }
        /*tet_number=1;
        Tet = new int[tet_number*4];
        Tet[0]=0;
        Tet[1]=1;
        Tet[2]=2;
        Tet[3]=3;

        number=4;
        X = new Vector3[number];
        V = new Vector3[number];
        Force = new Vector3[number];
        X[0]= new Vector3(0, 0, 0);
        X[1]= new Vector3(1, 0, 0);
        X[2]= new Vector3(0, 1, 0);
        X[3]= new Vector3(0, 0, 1);*/


        //Create triangle mesh.
        Vector3[] vertices = new Vector3[tet_number * 12];
        int vertex_number = 0;
        for (int tet = 0; tet < tet_number; tet++)
        {
            vertices[vertex_number++] = X[Tet[tet * 4 + 0]];
            vertices[vertex_number++] = X[Tet[tet * 4 + 2]];
            vertices[vertex_number++] = X[Tet[tet * 4 + 1]];

            vertices[vertex_number++] = X[Tet[tet * 4 + 0]];
            vertices[vertex_number++] = X[Tet[tet * 4 + 3]];//4
            vertices[vertex_number++] = X[Tet[tet * 4 + 2]];

            vertices[vertex_number++] = X[Tet[tet * 4 + 0]];
            vertices[vertex_number++] = X[Tet[tet * 4 + 1]];
            vertices[vertex_number++] = X[Tet[tet * 4 + 3]];//8

            vertices[vertex_number++] = X[Tet[tet * 4 + 1]];
            vertices[vertex_number++] = X[Tet[tet * 4 + 2]];
            vertices[vertex_number++] = X[Tet[tet * 4 + 3]];//11
        }

        int[] triangles = new int[tet_number * 12];
        for (int t = 0; t < tet_number * 4; t++)
        {
            triangles[t * 3 + 0] = t * 3 + 0;
            triangles[t * 3 + 1] = t * 3 + 1;
            triangles[t * 3 + 2] = t * 3 + 2;
        }
        Mesh mesh = GetComponent<MeshFilter>().mesh;
        mesh.vertices = vertices;
        mesh.triangles = triangles;
        mesh.RecalculateNormals();


        V = new Vector3[number];
        Force = new Vector3[number];
        V_sum = new Vector3[number];
        V_num = new int[number];
        
        //TODO: Need to allocate and assign inv_Dm
        inv_Dm = new Matrix4x4[tet_number];
        for (int i = 0; i < tet_number; ++i)
        {
            inv_Dm[i] = Build_Edge_Matrix(i).inverse;
            //  Debug.Log(inv_Dm[i]);
        }


        graph = new bool[number][];
        for (int i = 0; i < number; ++i)
        {
            graph[i] = new bool[number];
        }
        for (int i = 0; i < Tet.Length; i += 4)
        {
            graph[Tet[i]][Tet[i + 1]] = true;
            graph[Tet[i + 1]][Tet[i]] = true;
            graph[Tet[i]][Tet[i + 2]] = true;
            graph[Tet[i + 2]][Tet[i]] = true;
            graph[Tet[i]][Tet[i + 3]] = true;
            graph[Tet[i + 3]][Tet[i]] = true;
            graph[Tet[i + 2]][Tet[i + 1]] = true;
            graph[Tet[i + 1]][Tet[i + 2]] = true;
            graph[Tet[i + 2]][Tet[i + 3]] = true;
            graph[Tet[i + 3]][Tet[i + 2]] = true;
            graph[Tet[i + 1]][Tet[i + 3]] = true;
            graph[Tet[i + 3]][Tet[i + 1]] = true;
        }
        for(int i = 0;i < graph.Length; ++i)
        {
            for(int j = 0;j < graph[i].Length; ++j)
            {
                if(graph[i][j])
                {
                    V_num[i] += 1;
                }
            }
        }
        neighbors = new int[number][];
        for(int i = 0;i < number; ++i)
        {
            neighbors[i] = new int[V_num[i]];
            int cur = 0;
            for(int j = 0;j < graph[i].Length; ++j)
            {
                if(graph[i][j])
                {
                    neighbors[i][cur] = j;
                    cur += 1;
                }
            }
        }
    }

    Matrix4x4 Build_Edge_Matrix(int tet)
    {
        Matrix4x4 ret = Matrix4x4.zero;
        //TODO: Need to build edge matrix here.
        Vector3 X10 = X[Tet[4 * tet + 1]] - X[Tet[4 * tet]];
        Vector3 X20 = X[Tet[4 * tet + 2]] - X[Tet[4 * tet]];
        Vector3 X30 = X[Tet[4 * tet + 3]] - X[Tet[4 * tet]];
        ret.SetColumn(0, X10);
        ret.SetColumn(1, X20);
        ret.SetColumn(2, X30);
        /*
        ret[0, 0] = X10[0];
        ret[1, 0] = X10[1];
        ret[2, 0] = X10[2];
        ret[0, 1] = X20[0];
        ret[1, 1] = X20[1];
        ret[2, 1] = X20[2];
        ret[0, 2] = X30[0];
        ret[1, 2] = X30[1];
        ret[2, 2] = X30[2];
        */
        ret[3, 3] = 1.0f;
        return ret;
    }

    void ImpluseCollosion()
    {
        for(int surface = 0;surface < P.Length; ++surface)
        {
            for(int i = 0;i < X.Length; ++i)
            {
                if(Vector3.Dot(X[i] - P[surface], N[surface]) < 0)
                {
                    if(Vector3.Dot(V[i], N[surface]) < 0)
                    {
                        Vector3 Vn = Vector3.Dot(V[i], N[surface]) * N[surface];
                        Vector3 Vt = V[i] - Vn;
                        float a = Mathf.Max(1 - muT * (1 + muN) * Vn.magnitude / Vt.magnitude, 0);
                        Vn *= -muN;
                        Vt *= a;
                        V[i] = Vn + Vt;
                    }
                    X[i] -= Vector3.Dot(X[i] - P[surface], N[surface]) * N[surface];
                }
            }
        }
    }
    void _Update()
    {
        // Jump up.
        if (Input.GetKeyDown(KeyCode.Space))
        {
            for (int i = 0; i < number; i++)
                V[i].y += 0.2f;
        }

        for (int i = 0; i < number; i++)
        {
            //TODO: Add gravity to Force.
            Vector3 gravity_f = new Vector3(0.0f, -9.8f, 0.0f);
            Force[i] = gravity_f;
        }

        for (int tet = 0; tet < tet_number; tet++)
        {
            //TODO: Deformation Gradient
            Matrix4x4 F = Build_Edge_Matrix(tet) * inv_Dm[tet];
            F[3, 3] = 1;

            //svd
           /* 
            Matrix4x4 U = Matrix4x4.zero;
            Matrix4x4 S = Matrix4x4.zero;
            Matrix4x4 VT = Matrix4x4.zero;
            SVD SVD = new SVD();
            SVD.svd(F, ref U, ref S, ref VT);
            VT = VT.transpose;

            // StVK model & Neo-Hookean model
            Vector3 lambda = new Vector3(S[0, 0], S[1, 1], S[2, 2]);    // lambda0, lambda1, lambda2
            Vector3 lambda_2 = Vector3.Scale(lambda, lambda);           // lambda0^2, lambda1^2, lambda2^2
            Vector3 Is = new Vector3(
                lambda[0] + lambda[1] + lambda[2],
                lambda_2[0] + lambda_2[1] + lambda_2[2],
                lambda[0] * lambda[1] * lambda[2]);                     // I, II, III
            Matrix4x4 D = Matrix4x4.identity;
            // StVK
             for (int i = 0; i < 3; i++)
            {
                 D[i, i] = 2 * stiffness_0 * S[i, i] * (Is[0] - 3) + stiffness_1 * S[i, i] * (S[i, i] * S[i, i] - 1);
             }
            Matrix4x4 P = U * D * VT;
            */
            Matrix4x4 U = Matrix4x4.zero;
            Matrix4x4 S = Matrix4x4.zero;
            Matrix4x4 V = Matrix4x4.zero;
            SVD SVD = new SVD();
            SVD.svd(F, ref U, ref S, ref V);
            Matrix4x4 VT = V.transpose;
            Vector3 lambda = new Vector3(S[0, 0], S[1, 1], S[2, 2]);
            float SquareSum = lambda[0] + lambda[1] + lambda[2];
            Matrix4x4 Diag = Matrix4x4.zero;
            for(int p = 0;p < 3; ++p)
            {
                Diag[p, p] = 2 * stiffness_0 * (SquareSum - 3) + stiffness_1 * (lambda[p] * lambda[p] - 1);
                Diag[p, p] *= Mathf.Pow(lambda[p], 0.5f);
            }
            Diag[3, 3] = 1;
            Matrix4x4 P = U * Diag * VT;
            
            /*
            //TODO: Green Strain
            Matrix4x4 G = F.transpose * F;

            G[0, 0] -= 1.0f;
            G[1, 1] -= 1.0f;
            G[2, 2] -= 1.0f;
            G[3, 3] -= 1.0f;
            for (int i = 0; i < 4; ++i)
            {
                for (int j = 0; j < 4; ++j)
                {
                    G[i, j] /= 2.0f;
                }
            }
            //TODO: Second PK Stress
            Matrix4x4 left = G;
            Matrix4x4 right = Matrix4x4.identity;
            float trace = G[0, 0] + G[1, 1] + G[2, 2];// + G[3,3];
            Matrix4x4 S = Matrix4x4.zero;
            for (int i = 0; i < 4; ++i)
            {
                for (int j = 0; j < 4; ++j)
                {
                    S[i, j] = 2 * stiffness_1 * left[i, j] + trace * stiffness_0 * right[i, j];
                }
            }
            Matrix4x4 P = F * S;
            */


            //TODO: Elastic Force
            Matrix4x4 ForceMatrix = P * inv_Dm[tet].transpose;
            ForceMatrix[3, 3] = 1;
            float mul = -1.0f / (6.0f * inv_Dm[tet].determinant);
            for (int i = 0; i < 4; ++i)
            {
                for (int j = 0; j < 4; ++j)
                {
                    ForceMatrix[i, j] *= mul;
                }
            }
            Vector3 f1 = ForceMatrix.GetColumn(0);//  new Vector3(ForceMatrix[0,0], ForceMatrix[1,0], ForceMatrix[2,0]);
            Vector3 f2 = ForceMatrix.GetColumn(1);//new Vector3(ForceMatrix[0,1], ForceMatrix[1,1], ForceMatrix[2,1]);
            Vector3 f3 = ForceMatrix.GetColumn(2);//new Vector3(ForceMatrix[0,2], ForceMatrix[1,2], ForceMatrix[2,2]);
            Vector3 f0 = -f1 - f2 - f3;
            Force[Tet[4 * tet]] += f0;
            Force[Tet[4 * tet + 1]] += f1;
            Force[Tet[4 * tet + 2]] += f2;
            Force[Tet[4 * tet + 3]] += f3;
        }

        for (int i = 0; i < number; i++)
        {
            //TODO: Update X and V here.
            V[i] += Force[i] * dt;
            V[i] *= damp;
            //      Debug.Log(V[i]);

            //Laplacian Smoothing
            V[i] = (V_sum[i] + V_num[i] * V[i]) / (2 * V_num[i]);
            
      //      Debug.Log(V_num[i]);
            

            X[i] += V[i] * dt;
            //TODO: (Particle) collision with floor.
            ImpluseCollosion();
        }
        for (int first = 0; first < number; ++first)
        {
            V_sum[first] = new Vector3(0.0f, 0.0f, 0.0f);
            for (int second = 0; second < neighbors[first].Length; ++second)
            {
                V_sum[first] += V[neighbors[first][second]];
            }
        }
    }

    // Update is called once per frame
    void Update()
    {
        for (int l = 0; l < 10; l++)
            _Update();

        // Dump the vertex array for rendering.
        Vector3[] vertices = new Vector3[tet_number * 12];
        int vertex_number = 0;
        for (int tet = 0; tet < tet_number; tet++)
        {
            vertices[vertex_number++] = X[Tet[tet * 4 + 0]];
            vertices[vertex_number++] = X[Tet[tet * 4 + 2]];
            vertices[vertex_number++] = X[Tet[tet * 4 + 1]];
            vertices[vertex_number++] = X[Tet[tet * 4 + 0]];
            vertices[vertex_number++] = X[Tet[tet * 4 + 3]];
            vertices[vertex_number++] = X[Tet[tet * 4 + 2]];
            vertices[vertex_number++] = X[Tet[tet * 4 + 0]];
            vertices[vertex_number++] = X[Tet[tet * 4 + 1]];
            vertices[vertex_number++] = X[Tet[tet * 4 + 3]];
            vertices[vertex_number++] = X[Tet[tet * 4 + 1]];
            vertices[vertex_number++] = X[Tet[tet * 4 + 2]];
            vertices[vertex_number++] = X[Tet[tet * 4 + 3]];
        }
        Mesh mesh = GetComponent<MeshFilter>().mesh;
        mesh.vertices = vertices;
        mesh.RecalculateNormals();
    }
}