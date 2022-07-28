using UnityEngine;
using System.Collections;

public class wave_motion : MonoBehaviour 
{
	int size 		= 100;
	float rate 		= 0.005f;
	float gamma		= 0.004f;
	float damping 	= 0.996f;
	float[,] 	old_h;
	float[,]	low_h;
	float[,]	vh;
	float[,]	b;

	bool [,]	cg_mask;
	float[,]	cg_p;
	float[,]	cg_r;
	float[,]	cg_Ap;
	bool 	tag=true;

	Vector3 	cube_v = Vector3.zero;
	Vector3 	cube_w = Vector3.zero;


	Matrix4x4 I_ref_cube;
	Vector3 V_cube = new Vector3(0.0f, 0.0f, 0.0f);
	Vector3 W_cube = new Vector3(0.0f, 0.0f, 0.0f);
	
	Matrix4x4 I_ref_block;
	Vector3 V_block = new Vector3(0.0f, 0.0f, 0.0f);
	Vector3 W_block = new Vector3(0.0f, 0.0f, 0.0f);

	float dt = 0.003f;
	//Vector3[,] Rri;
	float mass;

	void ClacIref()
    {
		GameObject block = GameObject.Find("Block");
		Vector3[] vertices1 = block.GetComponent<MeshFilter>().mesh.vertices;
		

		float m = 1;
		mass = 0;
		for (int i = 0; i < vertices1.Length; i++)
		{
			mass += m;
			float diag = m * vertices1[i].sqrMagnitude;
			I_ref_block[0, 0] += diag;
			I_ref_block[1, 1] += diag;
			I_ref_block[2, 2] += diag;
			I_ref_block[0, 0] -= m * vertices1[i][0] * vertices1[i][0];
			I_ref_block[0, 1] -= m * vertices1[i][0] * vertices1[i][1];
			I_ref_block[0, 2] -= m * vertices1[i][0] * vertices1[i][2];
			I_ref_block[1, 0] -= m * vertices1[i][1] * vertices1[i][0];
			I_ref_block[1, 1] -= m * vertices1[i][1] * vertices1[i][1];
			I_ref_block[1, 2] -= m * vertices1[i][1] * vertices1[i][2];
			I_ref_block[2, 0] -= m * vertices1[i][2] * vertices1[i][0];
			I_ref_block[2, 1] -= m * vertices1[i][2] * vertices1[i][1];
			I_ref_block[2, 2] -= m * vertices1[i][2] * vertices1[i][2];
		}
		I_ref_block[3, 3] = 1;

		GameObject cube = GameObject.Find("Cube");
		Vector3[] vertices2 = cube.GetComponent<MeshFilter>().mesh.vertices;


		m = 1;
		mass = 0;
		for (int i = 0; i < vertices2.Length; i++)
		{
			mass += m;
			float diag = m * vertices2[i].sqrMagnitude;
			I_ref_cube[0, 0] += diag;
			I_ref_cube[1, 1] += diag;
			I_ref_cube[2, 2] += diag;
			I_ref_cube[0, 0] -= m * vertices2[i][0] * vertices2[i][0];
			I_ref_cube[0, 1] -= m * vertices2[i][0] * vertices2[i][1];
			I_ref_cube[0, 2] -= m * vertices2[i][0] * vertices2[i][2];
			I_ref_cube[1, 0] -= m * vertices2[i][1] * vertices2[i][0];
			I_ref_cube[1, 1] -= m * vertices2[i][1] * vertices2[i][1];
			I_ref_cube[1, 2] -= m * vertices2[i][1] * vertices2[i][2];
			I_ref_cube[2, 0] -= m * vertices2[i][2] * vertices2[i][0];
			I_ref_cube[2, 1] -= m * vertices2[i][2] * vertices2[i][1];
			I_ref_cube[2, 2] -= m * vertices2[i][2] * vertices2[i][2];
		}
		I_ref_cube[3, 3] = 1;


	}

	// Use this for initialization
	void Start () 
	{
		Mesh mesh = GetComponent<MeshFilter> ().mesh;
		mesh.Clear ();

		Vector3[] X=new Vector3[size*size];

		for (int i=0; i<size; i++)
		for (int j=0; j<size; j++) 
		{
			X[i*size+j].x=i*0.1f-size*0.05f;
			X[i*size+j].y=0;
			X[i*size+j].z=j*0.1f-size*0.05f;
		}

		int[] T = new int[(size - 1) * (size - 1) * 6];
		int index = 0;
		for (int i=0; i<size-1; i++)
		for (int j=0; j<size-1; j++)
		{
			T[index*6+0]=(i+0)*size+(j+0);
			T[index*6+1]=(i+0)*size+(j+1);
			T[index*6+2]=(i+1)*size+(j+1);
			T[index*6+3]=(i+0)*size+(j+0);
			T[index*6+4]=(i+1)*size+(j+1);
			T[index*6+5]=(i+1)*size+(j+0);
			index++;
		}
		mesh.vertices  = X;
		mesh.triangles = T;
		mesh.RecalculateNormals ();

		low_h 	= new float[size,size];
		old_h 	= new float[size,size];
		vh 	  	= new float[size,size];
		b 	  	= new float[size,size];

		cg_mask	= new bool [size,size];
		cg_p 	= new float[size,size];
		cg_r 	= new float[size,size];
		cg_Ap 	= new float[size,size];

		for (int i=0; i<size; i++)
		for (int j=0; j<size; j++) 
		{
			low_h[i,j]=99999;
			old_h[i,j]=0;
			vh[i,j]=0;
		}
		ClacIref();

	}

	void A_Times(bool[,] mask, float[,] x, float[,] Ax, int li, int ui, int lj, int uj)
	{
		for(int i=li; i<=ui; i++)
		for(int j=lj; j<=uj; j++)
		if(i>=0 && j>=0 && i<size && j<size && mask[i,j])
		{
			Ax[i,j]=0;
			if(i!=0)		Ax[i,j]-=x[i-1,j]-x[i,j];
			if(i!=size-1)	Ax[i,j]-=x[i+1,j]-x[i,j];
			if(j!=0)		Ax[i,j]-=x[i,j-1]-x[i,j];
			if(j!=size-1)	Ax[i,j]-=x[i,j+1]-x[i,j];
		}
	}

	float Dot(bool[,] mask, float[,] x, float[,] y, int li, int ui, int lj, int uj)
	{
		float ret=0;
		for(int i=li; i<=ui; i++)
		for(int j=lj; j<=uj; j++)
		if(i>=0 && j>=0 && i<size && j<size && mask[i,j])
		{
			ret+=x[i,j]*y[i,j];
		}
		return ret;
	}

	void Conjugate_Gradient(bool[,] mask, float[,] b, float[,] x, int li, int ui, int lj, int uj)
	{
		//Solve the Laplacian problem by CG.
		A_Times(mask, x, cg_r, li, ui, lj, uj);

		for(int i=li; i<=ui; i++)
		for(int j=lj; j<=uj; j++)
		if(i>=0 && j>=0 && i<size && j<size && mask[i,j])
		{
			cg_p[i,j]=cg_r[i,j]=b[i,j]-cg_r[i,j];
		}

		float rk_norm=Dot(mask, cg_r, cg_r, li, ui, lj, uj);

		for(int k=0; k<128; k++)
		{
			if(rk_norm<1e-10f)	break;
			A_Times(mask, cg_p, cg_Ap, li, ui, lj, uj);
			float alpha=rk_norm/Dot(mask, cg_p, cg_Ap, li, ui, lj, uj);

			for(int i=li; i<=ui; i++)
			for(int j=lj; j<=uj; j++)
			if(i>=0 && j>=0 && i<size && j<size && mask[i,j])
			{
				x[i,j]   +=alpha*cg_p[i,j];
				cg_r[i,j]-=alpha*cg_Ap[i,j];
			}

			float _rk_norm=Dot(mask, cg_r, cg_r, li, ui, lj, uj);
			float beta=_rk_norm/rk_norm;
			rk_norm=_rk_norm;

			for(int i=li; i<=ui; i++)
			for(int j=lj; j<=uj; j++)
			if(i>=0 && j>=0 && i<size && j<size && mask[i,j])
			{
				cg_p[i,j]=cg_r[i,j]+beta*cg_p[i,j];
			}
		}

	}


	void Water2Block(string Name, float[,] new_h, ref Vector3 Torque, ref float Force)
    {
		Mesh mesh = GetComponent<MeshFilter>().mesh;
		Vector3[] X = mesh.vertices;
		GameObject block = GameObject.Find(Name);
		Collider collider = block.GetComponent<Collider>();
		Bounds bounds = collider.bounds;
		Vector3 pos = block.transform.position;
		Quaternion R = block.transform.rotation;
		//Vector3[] vertices1 = block1.GetComponent<MeshFilter>().mesh.vertices;
		Vector3[,] Rri = new Vector3[size, size];

		int li = (int)Mathf.Clamp((pos.x + 4.5f) / 0.1f, 0, size - 1);
		int ui = (int)Mathf.Clamp((pos.x + 5.5f) / 0.1f, 0, size - 1);
		int lj = (int)Mathf.Clamp((pos.z + 4.5f) / 0.1f, 0, size - 1);
		int uj = (int)Mathf.Clamp((pos.z + 5.5f) / 0.1f, 0, size - 1);

		for (int i = 0; i < size; ++i)
		{
			for (int j = 0; j < size; ++j)
			{
				cg_mask[i, j] = false;
				b[i, j] = 0;
				if (i >= li && i <= ui && j >= li && j <= uj)
				{
					Ray ray = new Ray(new Vector3(X[i * size + j][0], -2.0f, X[i * size + j][2]), Vector3.up);
					if (collider.Raycast(ray, out RaycastHit hitInfo, 2.0f))
					{
						if (hitInfo.distance < 2.0f)
						{
							low_h[i, j] = hitInfo.distance - 2.0f;
							b[i, j] = (new_h[i, j] - low_h[i, j]) / rate;
							cg_mask[i, j] = true;
							Rri[i, j] = hitInfo.point - pos;
						}
					}
				}
			}
		}

		Conjugate_Gradient(cg_mask, b, vh, li, ui, li, uj);


		for(int i = 0;i < size; ++i)
        {
			for(int j = 0;j < size; ++j)
            {
				if(cg_mask[i, j])
                {
					Force += vh[i, j];
					Torque += Vector3.Cross(Rri[i, j], new Vector3(0.0f, vh[i, j], 0.0f));
				}
            }
        }


	}

	void Shallow_Wave(float[,] old_h, float[,] h, float [,] new_h)
	{		
		//Step 1:
		//TODO: Compute new_h based on the shallow wave model.
		for(int i = 0;i < size; ++i)
        {
			for(int j = 0;j < size; ++j)
            {
				new_h[i, j] = h[i, j] + damping * (h[i, j] - old_h[i, j]);
				if(i != 0)
                {
					new_h[i, j] += rate * (h[i - 1, j] - h[i, j]);
                }
				if(i != size - 1)
                {
					new_h[i, j] += rate * (h[i + 1, j] - h[i, j]);
				}
				if(j != 0)
                {
					new_h[i, j] += rate * (h[i, j - 1] - h[i, j]);
				}
				if(j != size - 1)
                {
					new_h[i, j] += rate * (h[i, j + 1] - h[i, j]);
				}
            }
			
        }
		//Step 2: Block->Water coupling
		//TODO: for block 1, calculate low_h.
		//TODO: then set up b and cg_mask for conjugate gradient.
		//TODO: Solve the Poisson equation to obtain vh (virtual height).

		GameObject block = GameObject.Find("Block");
		Collider collider = block.GetComponent<Collider>();
		Bounds bounds = collider.bounds;
		Vector3 pos = block.transform.position;
		Quaternion R = block.transform.rotation;

		Vector3 TorqueBlock = new Vector3(0.0f, 0.0f, 0.0f);
		float ForceBlock = 0.0f;
		Water2Block("Block",new_h,ref TorqueBlock, ref ForceBlock);

		Matrix4x4 I = Matrix4x4.Rotate(R) * I_ref_block * Matrix4x4.Rotate(R).transpose;

		W_block += dt * I.inverse.MultiplyVector(TorqueBlock);
		W_block *= 0.8f;
		Quaternion diffq = new Quaternion(W_block.x * dt / 2, W_block.y * dt / 2, W_block.z * dt / 2, 0);
		diffq = diffq * R;
		R.x = R.x + diffq.x;
		R.y = R.y + diffq.y;
		R.z = R.z + diffq.z;
		R.w = R.w + diffq.w;
		block.transform.rotation = R.normalized;
		Vector3 F = new Vector3(0.0f, ForceBlock -1000.0f, 0.0f);
	//	Debug.Log(ForceBlock);
		V_block += dt * F;
		pos += V_block * dt;
	//	Debug.Log(pos);
		//block.transform.position = pos;

		//TODO: for block 2, calculate low_h.
		//TODO: then set up b and cg_mask for conjugate gradient.
		//TODO: Solve the Poisson equation to obtain vh (virtual height).


		GameObject cube = GameObject.Find("Cube");
		collider = cube.GetComponent<Collider>();
		bounds = collider.bounds;
		pos = block.transform.position;
		R = block.transform.rotation;
		Vector3 TorqueCube = new Vector3(0.0f, 0.0f, 0.0f);
		float ForceCube = 0.0f;
		Water2Block("Cube", new_h, ref TorqueCube, ref ForceCube);
		I = Matrix4x4.Rotate(R) * I_ref_cube * Matrix4x4.Rotate(R).transpose;

		W_cube += dt * I.inverse.MultiplyVector(TorqueCube);
		W_cube *= 0.8f;
		diffq = new Quaternion(W_cube.x * dt / 2, W_cube.y * dt / 2, W_cube.z * dt / 2, 0);
		diffq = diffq * R;
		R.x = R.x + diffq.x;
		R.y = R.y + diffq.y;
		R.z = R.z + diffq.z;
		R.w = R.w + diffq.w;
		//cube.transform.rotation = R.normalized;
		F = new Vector3(0.0f, ForceCube - 1000.0f, 0.0f);
		//Debug.Log(ForceBlock);
		V_cube += dt * F;
		pos += V_cube * dt;
	//	Debug.Log(pos);
		//block.transform.position = pos;

		//TODO: Diminish vh.
		for (int i = 0; i < size; ++i)
		{
			for (int j = 0; j < size; ++j)
			{
				vh[i, j] *= gamma;
			}
		}
		//TODO: Update new_h by vh.

		for (int i = 0;i < size; ++i)
        {
			for(int j = 0;j < size; ++j)
            {
				if (i != 0)
				{
					new_h[i, j] += rate * (vh[i - 1, j] - vh[i, j]);
				}
				if (i != size - 1)
				{
					new_h[i, j] += rate * (vh[i + 1, j] - vh[i, j]);
				}
				if (j != 0)
				{
					new_h[i, j] += rate * (vh[i, j - 1] - vh[i, j]);
				}
				if (j != size - 1)
				{
					new_h[i, j] += rate * (vh[i, j + 1] - vh[i, j]);
				}
			}
        }

		//Step 3
		//TODO: old_h <- h; h <- new_h;
		for (int i = 0; i < size; ++i)
		{
			for (int j = 0; j < size; ++j)
			{
				old_h[i, j] = h[i, j];
				h[i, j] = new_h[i, j];
			}
		}
		//Step 4: Water->Block coupling.
		//More TODO here.

	}
	

	// Update is called once per frame
	void Update () 
	{
		Mesh mesh = GetComponent<MeshFilter> ().mesh;
		Vector3[] X    = mesh.vertices;
		float[,] new_h = new float[size, size];
		float[,] h     = new float[size, size];

		//TODO: Load X.y into h.
		for(int i = 0;i < X.Length; ++i)
        {
			h[i / size, i % size] = X[i][1];
        }
		if (Input.GetKeyDown ("r")) 
		{
			//TODO: Add random water.
			int i = Random.Range(1, size - 1);
			int j = Random.Range(1, size - 1);
			float r = Random.Range(0, 1f);
			h[i, j - 1] -= r / 4;
			h[i, j + 1] -= r / 4;
			h[i + 1, j] -= r / 4;
			h[i - 1, j] -= r / 4;
			h[i, j] += r;
		//	Debug.Log(r);
		}
	
		for(int l=0; l<8; l++)
		{
			Shallow_Wave(old_h, h, new_h);
		}

		//TODO: Store h back into X.y and recalculate normal.
		for (int i = 0; i < X.Length; ++i)
		{
			X[i][1] = h[i / size, i % size];
		}
		mesh.vertices = X;
		mesh.RecalculateNormals();

	}
}
