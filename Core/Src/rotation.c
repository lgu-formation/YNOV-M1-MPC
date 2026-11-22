#include "rotation.h"
#include <math.h>

static RotationMatrix3 produit_matrices(const RotationMatrix3 *A,
                                        const RotationMatrix3 *B)
{
    RotationMatrix3 C = {{{0.0f}}};
    int i, j, k;
    for (i = 0; i < 3; ++i)
    {
        for (j = 0; j < 3; ++j)
        {
            for (k = 0; k < 3; ++k)
            {
                C.m[i][j] += A->m[i][k] * B->m[k][j];
            }
        }
    }
    return C;
}

static RotationVector3 produit_matrice_vecteur(const RotationMatrix3 *R,
                                                RotationVector3 v)
{
    RotationVector3 result;
    result.x = R->m[0][0] * v.x + R->m[0][1] * v.y + R->m[0][2] * v.z;
    result.y = R->m[1][0] * v.x + R->m[1][1] * v.y + R->m[1][2] * v.z;
    result.z = R->m[2][0] * v.x + R->m[2][1] * v.y + R->m[2][2] * v.z;
    return result;
}

/* TODO MAT : écrire la matrice Rx(phi) */
static RotationMatrix3 construire_Rx(float phi)
{
    const float c = cosf(phi);
    const float s = sinf(phi);
    RotationMatrix3 R = {{{0.0f}}};
    (void)c; // pour éviter les warnings avant implémentation
    (void)s; // pour éviter les warnings avant implémentation
    /* TODO MAT-1 : initialiser les neuf coefficients de R.m[][] :
     * axe X inchangé, rotation dans le plan YZ.
     * Exemple : R.m[0][0] = 1.0f;
     */
    return R;
}

/* TODO : écrire la matrice Ry(theta). */
static RotationMatrix3 construire_Ry(float theta)
{
    const float c = cosf(theta);
    const float s = sinf(theta);
    RotationMatrix3 R = {{{0.0f}}};
    (void)c; //pour éviter les warnings avant implémentation
    (void)s; //pour éviter les warnings avant implémentation
    /* TODO : initialiser les neuf coefficients R.m[][] :
     * axe Y inchangé, rotation dans le plan XZ.
     */
    return R;
}

/* TODO : écrire la matrice Rz(psi). */
static RotationMatrix3 construire_Rz(float psi)
{
    const float c = cosf(psi);
    const float s = sinf(psi);
    RotationMatrix3 R = {{{0.0f}}};
    (void)c; //pour éviter les warnings avant implémentation
    (void)s; //pour éviter les warnings avant implémentation
    /* TODO : initialiser les neuf coefficients R.m[][] :
     * axe Z inchangé, rotation dans le plan XY.
     */
    return R;
}

RotationVector3 Rotation_ExprimerDansF(float phi_rad,
                                       float theta_rad,
                                       float psi_rad,
                                       RotationVector3 a_C)
{
    const RotationMatrix3 Rx = construire_Rx(phi_rad);
    const RotationMatrix3 Ry = construire_Ry(theta_rad);
    const RotationMatrix3 Rz = construire_Rz(psi_rad);

    const RotationMatrix3 etape = produit_matrices(&Rx, &Ry);
    const RotationMatrix3 R_FC = produit_matrices(&etape, &Rz);

    /* Nouvelles coordonnées : a_F = R_FC * a_C */
    return produit_matrice_vecteur(&R_FC, a_C);
}
