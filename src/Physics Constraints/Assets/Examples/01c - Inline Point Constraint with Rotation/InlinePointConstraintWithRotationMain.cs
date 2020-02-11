/******************************************************************************/
/*
  Project - Physics Constraints
            https://github.com/TheAllenChou/unity-physics-constraints
  
  Author  - Ming-Lun "Allen" Chou
  Web     - http://AllenChou.net
  Twitter - @TheAllenChou
*/
/******************************************************************************/

using UnityEngine;

using PhysicsConstraints;
using CjLib;

public class InlinePointConstraintWithRotationMain : MonoBehaviour {
	public float Beta = 0.02f;
	public int VelocityIterations = 10;

	public GameObject Box;
	public GameObject Target;

	public Vector3 Gravity = new Vector3(0.0f, -9.8f, 0.0f);
	public bool UseAngularDisplacementConstraint = true;
	public bool LockRotationX;
	public bool LockRotationY;
	public bool LockRotationZ;

	private float mass;
	private float massInv;
	private Matrix3x3 inertia;
	private Matrix3x3 inertiaInv;

	private Vector3 rLocal = Vector3.zero; // corner offset
	// Variables of Position Constraint
	private Vector3 posR = Vector3.zero;
	private Vector3 posC = Vector3.zero;
	// Variables of Rotation Constraint
	private Vector3 rotC = Vector3.zero;

	private Vector3 v = Vector3.zero; // linear velocity
	private Vector3 a = Vector3.zero; // angular velocity


	private void Start() {
		mass = 1.0f;
		massInv = 1.0f / mass;

		inertia = Inertia.SolidBox(mass, 1.0f * Vector3.one);
		inertiaInv = inertia.Inverted;

		rLocal = 0.5f * Vector3.one;
	}

	private void Update() {
		if (Box == null)
			return;

		if (Target == null)
			return;

		float dt = Time.deltaTime;

		// gravity
		v += Gravity * dt;

		InitPositionConstraint();
		InitRotationConstraint();

		// Solve.
		for (int i = 0; i < VelocityIterations; i++) {
			SolvePositionConstraint(dt);
			SolveRotationConstraint(dt);
		}

		v *= 0.98f; // temp magic cheat
		a *= 0.98f; // temp magic cheat
		// integration
		Box.transform.position += v * dt;
		Quaternion q = QuaternionUtil.AxisAngle(VectorUtil.NormalizeSafe(a, Vector3.forward), a.magnitude * dt);
		Box.transform.rotation = q * Box.transform.rotation;
	}

	private void InitPositionConstraint() {
		posR = Box.transform.rotation * rLocal;
		posC = (Box.transform.position + posR) - Target.transform.position;
	}

	private void InitRotationConstraint() {
		Box.transform.rotation.ToAngleAxis(out float qAngle, out Vector3 qAxis);
		rotC = Vector3.zero;
		if (!UseAngularDisplacementConstraint) { return; }

		Vector3 lockAngle = Mathf.Deg2Rad * qAngle * qAxis;
		if (LockRotationX) {
			rotC.x = lockAngle.x;
		}
		if (LockRotationY) {
			rotC.y = lockAngle.y;
		}
		if (LockRotationZ) {
			rotC.z = lockAngle.z;
		}
	}

	private void SolvePositionConstraint(float dt) {
		Vector3 jV = v + Vector3.Cross(a, posR);

		// constraint resolution
		Matrix3x3 s = Matrix3x3.PostCross(posR);
		Matrix3x3 k = massInv * Matrix3x3.Identity + s * inertiaInv * s.Transposed;
		Matrix3x3 effectiveMass = k.Inverted;
		Vector3 lambda = effectiveMass * (-(jV + (Beta / dt) * posC));

		// velocity correction
		v += massInv * lambda;
		a += (inertiaInv * s.Transposed) * lambda;

		//v *= 0.98f; // temp magic cheat
		//a *= 0.98f; // temp magic cheat
	}

	private void SolveRotationConstraint(float dt) {
		Vector3 jV = Vector3.zero;
		if (LockRotationX) {
			jV.x = a.x;
		}
		if (LockRotationY) {
			jV.y = a.y;
		}
		if (LockRotationZ) {
			jV.z = a.z;
		}
		Matrix3x3 effectiveMass = inertia;
		Vector3 lambda = effectiveMass * (-(jV + (Beta / dt) * rotC));
		a += inertiaInv * lambda;

		//a *= 0.98f; // temp magic cheat
	}
}
