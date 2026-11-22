#ifndef ROTATION_H
#define ROTATION_H


typedef struct { float x, y, z; } RotationVector3;
typedef struct { float m[3][3]; } RotationMatrix3;

/* F : repere fixe ; C : repere lie a la carte inertielle.
 * a_F = R_FC * a_C, avec R_FC = Rz(psi) * Ry(theta) * Rx(phi).
 * Angles en RADIANS. Les composantes a_C et a_F gardent la meme unite.
 */
RotationVector3 Rotation_ExprimerDansF(float phi_rad,
                                       float theta_rad,
                                       float psi_rad,
                                       RotationVector3 a_C);

#endif
