
#ifndef GVF_PARAMETRIC_2D_BEZIER_SPLINES_H
#define GVF_PARAMETRIC_2D_BEZIER_SPLINES_H



#ifndef GVF_PARAMETRIC_2D_BEZIER_N_SEG
#define GVF_PARAMETRIC_2D_BEZIER_N_SEG 4
#endif


#ifdef __cplusplus
extern "C" {
#endif


typedef struct {
	float kx;
	float ky;
}gvf_par_2d_bezier_par;


// Cubic bezier
typedef struct{
	float p0[2];
	float p1[2];
	float p2[2];
	float p3[2];
}bezier_t;


extern gvf_par_2d_bezier_par gvf_parametric_2d_bezier_par;

extern void create_bezier_spline(bezier_t *bezier, float *px, float *py);
extern void gvf_parametric_2d_bezier_splines_info(bezier_t *bezier, float *f1, float *f2, float *f1d, float *f2d, float *f1dd, float *f2dd);

#ifdef __cplusplus
}
#endif





#endif // bezier splines
