/*
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *                                                                     *
 * The ACME project                                                    *
 *                                                                     *
 * Copyright (c) 2020, Davide Stocco and Enrico Bertolazzi.            *
 *                                                                     *
 * The ACME project and its components are supplied under the terms of *
 * the open source BSD 2-Clause License. The contents of the ACME      *
 * project and its components may not be copied or disclosed except in *
 * accordance with the terms of the BSD 2-Clause License.              *
 *                                                                     *
 * URL: https://opensource.org/licenses/BSD-2-Clause                   *
 *                                                                     *
 *    Davide Stocco                                                    *
 *    Department of Industrial Engineering                             *
 *    University of Trento                                             *
 *    e-mail: davide.stocco@unitn.it                                   *
 *                                                                     *
 *    Enrico Bertolazzi                                                *
 *    Department of Industrial Engineering                             *
 *    University of Trento                                             *
 *    e-mail: enrico.bertolazzi@unitn.it                               *
 *                                                                     *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
*/

///
/// file: mex_ray.cc
///

#include "Utils_mex.hh"
#include "acme.hh"

#define ASSERT(COND, MSG)              \
  if (!(COND))                         \
  {                                    \
    std::ostringstream ost;            \
    ost << "mex_ray: " << MSG << '\n'; \
    mexErrMsgTxt(ost.str().c_str());   \
  }

#define MEX_ERROR_MESSAGE                                                     \
  "%=====================================================================%\n" \
  "% mex_ray: Mex wrapper for ACME ray object.                           %\n" \
  "%                                                                     %\n" \
  "% CONSTRUCTORS:                                                       %\n" \
  "%   obj = mex_ray( 'new' );                                           %\n" \
  "%   obj = mex_ray( 'new',                                             %\n" \
  "%                   [X; Y; Z], : Ray origin                           %\n" \
  "%                   [X; Y; Z]  : Ray direction                        %\n" \
  "%                 );                                                  %\n" \
  "%   obj = mex_ray( 'new',                                             %\n" \
  "%                   OX,   : Ray origin x value                        %\n" \
  "%                   OY,   : Ray origin y value                        %\n" \
  "%                   OZ,   : Ray origin z value                        %\n" \
  "%                   DX,   : Ray directon x value                      %\n" \
  "%                   DY,   : Ray directon y value                      %\n" \
  "%                   DZ    : Ray directon z value                      %\n" \
  "%                 );                                                  %\n" \
  "%                                                                     %\n" \
  "% DESTRUCTOR:                                                         %\n" \
  "%   mex_ray( 'delete', OBJ );                                         %\n" \
  "%                                                                     %\n" \
  "% USAGE:                                                              %\n" \
  "%   OUT = mex_ray( 'getOrigin', OBJ );                                %\n" \
  "%   OUT = mex_ray( 'getDirection', OBJ );                             %\n" \
  "%         mex_ray( 'setOrigin', OBJ, OTHER_OBJ );                     %\n" \
  "%         mex_ray( 'setDirection', OBJ, OTHER_OBJ );                  %\n" \
  "%         mex_ray( 'translate', OBJ, VECTOR );                        %\n" \
  "%         mex_ray( 'transform', OBJ, MATRIX );                        %\n" \
  "%         mex_ray( 'copy', OBJ, OTHER_OBJ );                          %\n" \
  "%   OUT = mex_ray( 'isInside', OBJ, OTHER_OBJ );                      %\n" \
  "%   OUT = mex_ray( 'isDegenerated', OBJ );                            %\n" \
  "%   OUT = mex_ray( 'isApprox', OBJ, OTHER_OBJ );                      %\n" \
  "%         mex_ray( 'normalize', OBJ );                                %\n" \
  "%   OUT = mex_ray( 'toVector', OBJ );                                 %\n" \
  "%   OUT = mex_ray( 'toUnitVector', OBJ );                             %\n" \
  "%         mex_ray( 'reverse', OBJ );                                  %\n" \
  "%   OUT = mex_ray( 'isParallel', OBJ, OTHER_OBJ );                    %\n" \
  "%   OUT = mex_ray( 'isOrthogonal', OBJ, OTHER_OBJ );                  %\n" \
  "%   OUT = mex_ray( 'isCollinear', OBJ, OTHER_OBJ );                   %\n" \
  "%   OUT = mex_ray( 'isCoplanar', OBJ, OTHER_OBJ );                    %\n" \
  "%   OUT = mex_ray( 'intersection', OBJ, OTHER_OBJ, TYPE );            %\n" \
  "%                                                                     %\n" \
  "%=====================================================================%\n" \
  "%                                                                     %\n" \
  "%    Davide Stocco                                                    %\n" \
  "%    Department of Industrial Engineering                             %\n" \
  "%    University of Trento                                             %\n" \
  "%    davide.stocco@unitn.it                                           %\n" \
  "%                                                                     %\n" \
  "%    Enrico Bertolazzi                                                %\n" \
  "%    Department of Industrial Engineering                             %\n" \
  "%    University of Trento                                             %\n" \
  "%    enrico.bertolazzi@unitn.it                                       %\n" \
  "%                                                                     %\n" \
  "%=====================================================================%\n"

using namespace std;

typedef double real_mex;

static void
DATA_NEW(
    mxArray *&mx_id,
    acme::ray *ptr)
{
  mx_id = Utils::mex_convert_ptr_to_mx<acme::ray>(ptr);
}

static inline acme::ray *
DATA_GET(
    mxArray const *&mx_id)
{
  return Utils::mex_convert_mx_to_ptr<acme::ray>(mx_id);
}

static void
DATA_DELETE(
    mxArray const *&mx_id)
{
  Utils::mex_destroy_object<acme::ray>(mx_id);
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

static void
do_new(int nlhs, mxArray *plhs[],
       int nrhs, mxArray const *prhs[])
{
#define CMD "mex_ray( 'new', [, args] ): "
  UTILS_MEX_ASSERT(nrhs == 1 || nrhs == 3 || nrhs == 7, CMD "expected 1, 3 or 7 inputs, nrhs = {}\n", nrhs);
  UTILS_MEX_ASSERT(nlhs == 1, CMD "expected 1 output, nlhs = {}\n", nlhs);

  UTILS_MEX_ASSERT(
      mxIsChar(arg_in_0),
      CMD "first argument must be a string, found ``{}''\n", mxGetClassName(arg_in_0));

  real_mex ox = acme::QUIET_NAN;
  real_mex oy = acme::QUIET_NAN;
  real_mex oz = acme::QUIET_NAN;
  real_mex dx = acme::QUIET_NAN;
  real_mex dy = acme::QUIET_NAN;
  real_mex dz = acme::QUIET_NAN;

  if (nrhs == 3)
  {
    real_mex const *matrix1_ptr;
    mwSize rows1, cols1;
    matrix1_ptr = Utils::mex_matrix_pointer(arg_in_1, rows1, cols1, CMD "Error in first input matrix");
    UTILS_MEX_ASSERT(rows1 == 3 || cols1 == 1, CMD "expected rows = 3 and cols = 1 found, rows = {}, cols = {}\n", rows1, cols1);
    ox = matrix1_ptr[0];
    oy = matrix1_ptr[1];
    oz = matrix1_ptr[2];
    real_mex const *matrix2_ptr;
    mwSize rows2, cols2;
    matrix2_ptr = Utils::mex_matrix_pointer(arg_in_2, rows2, cols2, CMD "Error in second input matrix");
    UTILS_MEX_ASSERT(rows2 == 3 || cols2 == 1, CMD "expected rows = 3 and cols = 1 found, rows = {}, cols = {}\n", rows2, cols2);
    dx = matrix2_ptr[0];
    dy = matrix2_ptr[1];
    dz = matrix2_ptr[2];
  }
  else if (nrhs == 7)
  {
    ox = Utils::mex_get_scalar_value(arg_in_1, CMD "Error in reading origin x value");
    oy = Utils::mex_get_scalar_value(arg_in_2, CMD "Error in reading origin y value");
    oz = Utils::mex_get_scalar_value(arg_in_3, CMD "Error in reading origin z value");
    dx = Utils::mex_get_scalar_value(arg_in_4, CMD "Error in reading direction x value");
    dy = Utils::mex_get_scalar_value(arg_in_5, CMD "Error in reading direction y value");
    dz = Utils::mex_get_scalar_value(arg_in_6, CMD "Error in reading direction z value");
  }

  acme::ray *ptr = new acme::ray(ox, oy, oz, dx, dy, dz);
  DATA_NEW(arg_out_0, ptr);
#undef CMD
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

static void
do_delete(int nlhs, mxArray *plhs[],
          int nrhs, mxArray const *prhs[])
{
#define CMD "mex_ray( 'delete', OBJ ): "
  UTILS_MEX_ASSERT(nrhs == 2, CMD "expected 2 inputs, nrhs = {}\n", nrhs);
  UTILS_MEX_ASSERT(nlhs == 0, CMD "expected 0 output, nlhs = {}\n", nlhs);

  DATA_DELETE(arg_in_1);
#undef CMD
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

static void
do_getOrigin(int nlhs, mxArray *plhs[],
             int nrhs, mxArray const *prhs[])
{
#define CMD "mex_ray( 'getOrigin', OBJ ): "
  UTILS_MEX_ASSERT(nrhs == 2, CMD "expected 2 inputs, nrhs = {}\n", nrhs);
  UTILS_MEX_ASSERT(nlhs == 1, CMD "expected 1 output, nlhs = {}\n", nlhs);

  acme::ray *self = DATA_GET(arg_in_1);
  acme::point *out = new acme::point(self->origin());
  arg_out_0 = Utils::mex_convert_ptr_to_mx<acme::point>(out);
#undef CMD
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

static void
do_getDirection(int nlhs, mxArray *plhs[],
                int nrhs, mxArray const *prhs[])
{
#define CMD "mex_ray( 'getDirection', OBJ ): "
  UTILS_MEX_ASSERT(nrhs == 2, CMD "expected 2 inputs, nrhs = {}\n", nrhs);
  UTILS_MEX_ASSERT(nlhs == 1, CMD "expected 1 output, nlhs = {}\n", nlhs);

  acme::ray *self = DATA_GET(arg_in_1);
  real_mex *output = Utils::mex_create_matrix_value(arg_out_0, 3, 1);
  acme::vec3 outvec(self->direction());
  output[0] = outvec.x();
  output[1] = outvec.y();
  output[2] = outvec.z();
#undef CMD
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

static void
do_setOrigin(int nlhs, mxArray *plhs[],
             int nrhs, mxArray const *prhs[])
{
#define CMD "mex_ray( 'setOrigin', OBJ, OTHER_OBJ ): "
  UTILS_MEX_ASSERT(nrhs == 3, CMD "expected 3 inputs, nrhs = {}\n", nrhs);
  UTILS_MEX_ASSERT(nlhs == 0, CMD "expected 0 output, nlhs = {}\n", nlhs);

  acme::ray *self = DATA_GET(arg_in_1);
  acme::point *obj = Utils::mex_convert_mx_to_ptr<acme::point>(arg_in_2);
  self->origin() = *obj;
#undef CMD
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

static void
do_setDirection(int nlhs, mxArray *plhs[],
                int nrhs, mxArray const *prhs[])
{
#define CMD "mex_ray( 'setDirection', OBJ, OTHER_OBJ ): "
  UTILS_MEX_ASSERT(nrhs == 3, CMD "expected 3 inputs, nrhs = {}\n", nrhs);
  UTILS_MEX_ASSERT(nlhs == 0, CMD "expected 0 output, nlhs = {}\n", nlhs);

  acme::ray *self = DATA_GET(arg_in_1);
  real_mex const *matrix_ptr;
  mwSize rows, cols;
  matrix_ptr = Utils::mex_matrix_pointer(arg_in_2, rows, cols, CMD "Error in first input matrix");
  UTILS_MEX_ASSERT(rows == 3 || cols == 1, CMD "expected rows = 3 and cols = 1 found, rows = {}, cols = {}\n", rows, cols);
  real_mex x = matrix_ptr[0];
  real_mex y = matrix_ptr[1];
  real_mex z = matrix_ptr[2];
  self->direction() = acme::vec3(x, y, z);
#undef CMD
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

static void
do_translate(int nlhs, mxArray *plhs[],
             int nrhs, mxArray const *prhs[])
{
#define CMD "mex_ray( 'translate', OBJ, [X; Y; Z] ): "
  UTILS_MEX_ASSERT(nrhs == 3, CMD "expected 3 inputs, nrhs = {}\n", nrhs);
  UTILS_MEX_ASSERT(nlhs == 0, CMD "expected 0 output, nlhs = {}\n", nlhs);

  acme::ray *self = DATA_GET(arg_in_1);
  real_mex const *matrix_ptr;
  mwSize rows, cols;
  matrix_ptr = Utils::mex_matrix_pointer(arg_in_2, rows, cols, CMD "Error in first input matrix");
  UTILS_MEX_ASSERT(rows == 3 || cols == 1, CMD "expected rows = 3 and cols = 1 found, rows = {}, cols = {}\n", rows, cols);
  real_mex x = matrix_ptr[0];
  real_mex y = matrix_ptr[1];
  real_mex z = matrix_ptr[2];
  self->translate(acme::vec3(x, y, z));
#undef CMD
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

static void
do_transform(int nlhs, mxArray *plhs[],
             int nrhs, mxArray const *prhs[])
{
#define CMD "mex_ray( 'transform', OBJ, MATRIX ): "
  UTILS_MEX_ASSERT(nrhs == 3, CMD "expected 3 inputs, nrhs = {}\n", nrhs);
  UTILS_MEX_ASSERT(nlhs == 0, CMD "expected 0 output, nlhs = {}\n", nlhs);

  acme::ray *self = DATA_GET(arg_in_1);
  real_mex const *matrix_ptr;
  mwSize rows, cols;
  matrix_ptr = Utils::mex_matrix_pointer(arg_in_2, rows, cols, CMD "Error in reading affine transformation matrix");
  acme::affine matrix;
  UTILS_MEX_ASSERT(rows == 4 || cols == 4, CMD "expected rows = 4 and cols = 4 found, rows = {}, cols = {}\n", rows, cols);
  matrix.matrix() << matrix_ptr[0], matrix_ptr[4], matrix_ptr[8], matrix_ptr[12],
      matrix_ptr[1], matrix_ptr[5], matrix_ptr[9], matrix_ptr[13],
      matrix_ptr[2], matrix_ptr[6], matrix_ptr[10], matrix_ptr[14],
      matrix_ptr[3], matrix_ptr[7], matrix_ptr[11], matrix_ptr[15];
  self->transform(matrix);
#undef CMD
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

static void
do_copy(int nlhs, mxArray *plhs[],
        int nrhs, mxArray const *prhs[])
{
#define CMD "mex_ray( 'copy', OBJ, OTHER_OBJ ): "
  UTILS_MEX_ASSERT(nrhs == 3, CMD "expected 3 inputs, nrhs = {}\n", nrhs);
  UTILS_MEX_ASSERT(nlhs == 0, CMD "expected 0 output, nlhs = {}\n", nlhs);

  acme::ray *self = DATA_GET(arg_in_1);
  acme::ray *other = DATA_GET(arg_in_2);
  *self = *other;
#undef CMD
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

static void
do_isInside(int nlhs, mxArray *plhs[],
            int nrhs, mxArray const *prhs[])
{
#define CMD "mex_ray( 'isInside', OBJ, OTHER_OBJ ): "
  UTILS_MEX_ASSERT(nrhs == 3, CMD "expected 3 inputs, nrhs = {}\n", nrhs);
  UTILS_MEX_ASSERT(nlhs == 1, CMD "expected 1 output, nlhs = {}\n", nlhs);

  acme::ray *self = DATA_GET(arg_in_1);
  acme::point *other = Utils::mex_convert_mx_to_ptr<acme::point>(arg_in_2);
  Utils::mex_set_scalar_bool(arg_out_0, self->isInside(*other));
#undef CMD
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

static void
do_isDegenerated(int nlhs, mxArray *plhs[],
                 int nrhs, mxArray const *prhs[])
{
#define CMD "mex_ray( 'isDegenerated', OBJ ): "
  UTILS_MEX_ASSERT(nrhs == 2, CMD "expected 2 inputs, nrhs = {}\n", nrhs);
  UTILS_MEX_ASSERT(nlhs == 1, CMD "expected 1 output, nlhs = {}\n", nlhs);

  acme::ray *self = DATA_GET(arg_in_1);
  Utils::mex_set_scalar_bool(arg_out_0, self->isDegenerated());
#undef CMD
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

static void
do_isApprox(int nlhs, mxArray *plhs[],
            int nrhs, mxArray const *prhs[])
{
#define CMD "mex_ray( 'isApprox', OBJ, OTHER_OBJ ): "
  UTILS_MEX_ASSERT(nrhs == 3, CMD "expected 3 inputs, nrhs = {}\n", nrhs);
  UTILS_MEX_ASSERT(nlhs == 1, CMD "expected 1 output, nlhs = {}\n", nlhs);

  acme::ray *self = DATA_GET(arg_in_1);
  acme::ray *other = DATA_GET(arg_in_2);
  Utils::mex_set_scalar_bool(arg_out_0, self->isApprox(*other));
#undef CMD
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

static void
do_normalize(int nlhs, mxArray *plhs[],
             int nrhs, mxArray const *prhs[])
{

#define CMD "mex_ray( 'normalize', OBJ ): "
  UTILS_MEX_ASSERT(nrhs == 2, CMD "expected 2 inputs, nrhs = {}\n", nrhs);
  UTILS_MEX_ASSERT(nlhs == 0, CMD "expected 0 output, nlhs = {}\n", nlhs);

  acme::ray *self = DATA_GET(arg_in_1);
  self->normalize();
#undef CMD
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

static void
do_toVector(int nlhs, mxArray *plhs[],
            int nrhs, mxArray const *prhs[])
{
#define CMD "mex_ray( 'toVector', OBJ ): "
  UTILS_MEX_ASSERT(nrhs == 2, CMD "expected 2 inputs, nrhs = {}\n", nrhs);
  UTILS_MEX_ASSERT(nlhs == 1, CMD "expected 1 output, nlhs = {}\n", nlhs);

  acme::ray *self = DATA_GET(arg_in_1);
  real_mex *output = Utils::mex_create_matrix_value(arg_out_0, 3, 1);
  acme::vec3 outvec(self->toVector());
  output[0] = outvec.x();
  output[1] = outvec.y();
  output[2] = outvec.z();
#undef CMD
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

static void
do_toUnitVector(int nlhs, mxArray *plhs[],
                int nrhs, mxArray const *prhs[])
{
#define CMD "mex_ray( 'toUnitVector', OBJ ): "
  UTILS_MEX_ASSERT(nrhs == 2, CMD "expected 2 inputs, nrhs = {}\n", nrhs);
  UTILS_MEX_ASSERT(nlhs == 1, CMD "expected 1 output, nlhs = {}\n", nlhs);

  acme::ray *self = DATA_GET(arg_in_1);
  real_mex *output = Utils::mex_create_matrix_value(arg_out_0, 3, 1);
  acme::vec3 outvec(self->toUnitVector());
  output[0] = outvec.x();
  output[1] = outvec.y();
  output[2] = outvec.z();
#undef CMD
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

static void
do_reverse(int nlhs, mxArray *plhs[],
           int nrhs, mxArray const *prhs[])
{
#define CMD "mex_ray( 'reverse', OBJ ): "
  UTILS_MEX_ASSERT(nrhs == 2, CMD "expected 2 inputs, nrhs = {}\n", nrhs);
  UTILS_MEX_ASSERT(nlhs == 0, CMD "expected 0 output, nlhs = {}\n", nlhs);

  acme::ray *self = DATA_GET(arg_in_1);
  self->reverse();
#undef CMD
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

static void
do_isParallel(int nlhs, mxArray *plhs[],
              int nrhs, mxArray const *prhs[])
{
#define CMD "mex_ray( 'isParallel', OBJ, OTHER_OBJ ): "
  UTILS_MEX_ASSERT(nrhs == 4, CMD "expected 4 inputs, nrhs = {}\n", nrhs);
  UTILS_MEX_ASSERT(nlhs == 1, CMD "expected 1 output, nlhs = {}\n", nlhs);

  acme::ray *self = DATA_GET(arg_in_1);
  string type = mxArrayToString(arg_in_3);
  acme::entity *other = nullptr;

  if (type == "none")
    other = Utils::mex_convert_mx_to_ptr<acme::none>(arg_in_2);
  else if (type == "point")
    other = Utils::mex_convert_mx_to_ptr<acme::point>(arg_in_2);
  else if (type == "line")
    other = Utils::mex_convert_mx_to_ptr<acme::line>(arg_in_2);
  else if (type == "ray")
    other = Utils::mex_convert_mx_to_ptr<acme::ray>(arg_in_2);
  else if (type == "plane")
    other = Utils::mex_convert_mx_to_ptr<acme::plane>(arg_in_2);
  else if (type == "segment")
    other = Utils::mex_convert_mx_to_ptr<acme::segment>(arg_in_2);
  else if (type == "triangle")
    other = Utils::mex_convert_mx_to_ptr<acme::triangle>(arg_in_2);
  else if (type == "disk")
    other = Utils::mex_convert_mx_to_ptr<acme::disk>(arg_in_2);
  else if (type == "ball")
    other = Utils::mex_convert_mx_to_ptr<acme::ball>(arg_in_2);

  Utils::mex_set_scalar_bool(arg_out_0, acme::IsParallel(self, other));
#undef CMD
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

static void
do_isOrthogonal(int nlhs, mxArray *plhs[],
                int nrhs, mxArray const *prhs[])
{
#define CMD "mex_ray( 'isOrthogonal', OBJ, OTHER_OBJ ): "
  UTILS_MEX_ASSERT(nrhs == 4, CMD "expected 4 inputs, nrhs = {}\n", nrhs);
  UTILS_MEX_ASSERT(nlhs == 1, CMD "expected 1 output, nlhs = {}\n", nlhs);

  acme::ray *self = DATA_GET(arg_in_1);
  string type = mxArrayToString(arg_in_3);
  acme::entity *other = nullptr;

  if (type == "none")
    other = Utils::mex_convert_mx_to_ptr<acme::none>(arg_in_2);
  else if (type == "point")
    other = Utils::mex_convert_mx_to_ptr<acme::point>(arg_in_2);
  else if (type == "line")
    other = Utils::mex_convert_mx_to_ptr<acme::line>(arg_in_2);
  else if (type == "ray")
    other = Utils::mex_convert_mx_to_ptr<acme::ray>(arg_in_2);
  else if (type == "plane")
    other = Utils::mex_convert_mx_to_ptr<acme::plane>(arg_in_2);
  else if (type == "segment")
    other = Utils::mex_convert_mx_to_ptr<acme::segment>(arg_in_2);
  else if (type == "triangle")
    other = Utils::mex_convert_mx_to_ptr<acme::triangle>(arg_in_2);
  else if (type == "disk")
    other = Utils::mex_convert_mx_to_ptr<acme::disk>(arg_in_2);
  else if (type == "ball")
    other = Utils::mex_convert_mx_to_ptr<acme::ball>(arg_in_2);

  Utils::mex_set_scalar_bool(arg_out_0, acme::IsOrthogonal(self, other));
#undef CMD
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

static void
do_isCollinear(int nlhs, mxArray *plhs[],
               int nrhs, mxArray const *prhs[])
{
#define CMD "mex_ray( 'isCollinear', OBJ, OTHER_OBJ ): "
  UTILS_MEX_ASSERT(nrhs == 4, CMD "expected 4 inputs, nrhs = {}\n", nrhs);
  UTILS_MEX_ASSERT(nlhs == 1, CMD "expected 1 output, nlhs = {}\n", nlhs);

  acme::ray *self = DATA_GET(arg_in_1);
  string type = mxArrayToString(arg_in_3);

  acme::entity *other = nullptr;

  if (type == "none")
    other = Utils::mex_convert_mx_to_ptr<acme::none>(arg_in_2);
  else if (type == "point")
    other = Utils::mex_convert_mx_to_ptr<acme::point>(arg_in_2);
  else if (type == "line")
    other = Utils::mex_convert_mx_to_ptr<acme::line>(arg_in_2);
  else if (type == "ray")
    other = Utils::mex_convert_mx_to_ptr<acme::ray>(arg_in_2);
  else if (type == "plane")
    other = Utils::mex_convert_mx_to_ptr<acme::plane>(arg_in_2);
  else if (type == "segment")
    other = Utils::mex_convert_mx_to_ptr<acme::segment>(arg_in_2);
  else if (type == "triangle")
    other = Utils::mex_convert_mx_to_ptr<acme::triangle>(arg_in_2);
  else if (type == "disk")
    other = Utils::mex_convert_mx_to_ptr<acme::disk>(arg_in_2);
  else if (type == "ball")
    other = Utils::mex_convert_mx_to_ptr<acme::ball>(arg_in_2);

  Utils::mex_set_scalar_bool(arg_out_0, acme::IsCollinear(self, other));
#undef CMD
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

static void
do_isCoplanar(int nlhs, mxArray *plhs[],
              int nrhs, mxArray const *prhs[])
{
#define CMD "mex_ray( 'isCoplanar', OBJ, OTHER_OBJ ): "
  UTILS_MEX_ASSERT(nrhs == 4, CMD "expected 4 inputs, nrhs = {}\n", nrhs);
  UTILS_MEX_ASSERT(nlhs == 1, CMD "expected 1 output, nlhs = {}\n", nlhs);

  acme::ray *self = DATA_GET(arg_in_1);
  string type = mxArrayToString(arg_in_3);

  acme::entity *other = nullptr;

  if (type == "none")
    other = Utils::mex_convert_mx_to_ptr<acme::none>(arg_in_2);
  else if (type == "point")
    other = Utils::mex_convert_mx_to_ptr<acme::point>(arg_in_2);
  else if (type == "line")
    other = Utils::mex_convert_mx_to_ptr<acme::line>(arg_in_2);
  else if (type == "ray")
    other = Utils::mex_convert_mx_to_ptr<acme::ray>(arg_in_2);
  else if (type == "plane")
    other = Utils::mex_convert_mx_to_ptr<acme::plane>(arg_in_2);
  else if (type == "segment")
    other = Utils::mex_convert_mx_to_ptr<acme::segment>(arg_in_2);
  else if (type == "triangle")
    other = Utils::mex_convert_mx_to_ptr<acme::triangle>(arg_in_2);
  else if (type == "disk")
    other = Utils::mex_convert_mx_to_ptr<acme::disk>(arg_in_2);
  else if (type == "ball")
    other = Utils::mex_convert_mx_to_ptr<acme::ball>(arg_in_2);

  Utils::mex_set_scalar_bool(arg_out_0, acme::IsCoplanar(self, other));
#undef CMD
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

static void
do_intersection(int nlhs, mxArray *plhs[],
                int nrhs, mxArray const *prhs[])
{
#define CMD "mex_ray( 'intersection', OBJ, OTHER_OBJ, TYPE ): "
  UTILS_MEX_ASSERT(nrhs == 4, CMD "expected 4 inputs, nrhs = {}\n", nrhs);
  UTILS_MEX_ASSERT(nlhs == 2, CMD "expected 2 output, nlhs = {}\n", nlhs);

  acme::ray *self = DATA_GET(arg_in_1);
  string type = mxArrayToString(arg_in_3);

  acme::entity *other = nullptr;

  if (type == "none")
    other = Utils::mex_convert_mx_to_ptr<acme::none>(arg_in_2);
  else if (type == "point")
    other = Utils::mex_convert_mx_to_ptr<acme::point>(arg_in_2);
  else if (type == "line")
    other = Utils::mex_convert_mx_to_ptr<acme::line>(arg_in_2);
  else if (type == "ray")
    other = Utils::mex_convert_mx_to_ptr<acme::ray>(arg_in_2);
  else if (type == "plane")
    other = Utils::mex_convert_mx_to_ptr<acme::plane>(arg_in_2);
  else if (type == "segment")
    other = Utils::mex_convert_mx_to_ptr<acme::segment>(arg_in_2);
  else if (type == "triangle")
    other = Utils::mex_convert_mx_to_ptr<acme::triangle>(arg_in_2);
  else if (type == "disk")
    other = Utils::mex_convert_mx_to_ptr<acme::disk>(arg_in_2);
  else if (type == "ball")
    other = Utils::mex_convert_mx_to_ptr<acme::ball>(arg_in_2);

  acme::entity *out = acme::Intersection(self, other);
  string out_type = out->type();
  if (out_type == "none")
    arg_out_0 = Utils::mex_convert_ptr_to_mx<acme::none>(dynamic_cast<acme::none *>(out));
  else if (out_type == "point")
    arg_out_0 = Utils::mex_convert_ptr_to_mx<acme::point>(dynamic_cast<acme::point *>(out));
  else if (out_type == "line")
    arg_out_0 = Utils::mex_convert_ptr_to_mx<acme::line>(dynamic_cast<acme::line *>(out));
  else if (out_type == "ray")
    arg_out_0 = Utils::mex_convert_ptr_to_mx<acme::ray>(dynamic_cast<acme::ray *>(out));
  else if (out_type == "plane")
    arg_out_0 = Utils::mex_convert_ptr_to_mx<acme::plane>(dynamic_cast<acme::plane *>(out));
  else if (out_type == "segment")
    arg_out_0 = Utils::mex_convert_ptr_to_mx<acme::segment>(dynamic_cast<acme::segment *>(out));
  else if (out_type == "triangle")
    arg_out_0 = Utils::mex_convert_ptr_to_mx<acme::triangle>(dynamic_cast<acme::triangle *>(out));
  else if (out_type == "disk")
    arg_out_0 = Utils::mex_convert_ptr_to_mx<acme::disk>(dynamic_cast<acme::disk *>(out));
  else if (out_type == "ball")
    arg_out_0 = Utils::mex_convert_ptr_to_mx<acme::ball>(dynamic_cast<acme::ball *>(out));

  arg_out_1 = mxCreateString(out_type.c_str());
#undef CMD
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

typedef void (*DO_CMD)(int nlhs, mxArray *plhs[],
                       int nrhs, mxArray const *prhs[]);

static map<string, DO_CMD> cmd_to_fun = {
    {"new", do_new},
    {"delete", do_delete},
    {"getOrigin", do_getOrigin},
    {"getDirection", do_getDirection},
    {"setOrigin", do_setOrigin},
    {"setDirection", do_setDirection},
    {"copy", do_copy},
    {"translate", do_translate},
    {"transform", do_transform},
    {"isInside", do_isInside},
    {"isDegenerated", do_isDegenerated},
    {"isApprox", do_isApprox},
    {"normalize", do_normalize},
    {"toVector", do_toVector},
    {"toUnitVector", do_toUnitVector},
    {"reverse", do_reverse},
    {"isParallel", do_isParallel},
    {"isOrthogonal", do_isOrthogonal},
    {"isCollinear", do_isCollinear},
    {"isCoplanar", do_isCoplanar},
    {"intersection", do_intersection}};

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

extern "C" void
mexFunction(int nlhs, mxArray *plhs[],
            int nrhs, mxArray const *prhs[])
{
  // First argument must be a string
  if (nrhs == 0)
  {
    mexErrMsgTxt(MEX_ERROR_MESSAGE);
    return;
  }

  try
  {
    UTILS_MEX_ASSERT0(mxIsChar(arg_in_0), "First argument must be a string");
    string cmd = mxArrayToString(arg_in_0);
    DO_CMD pfun = cmd_to_fun.at(cmd);
    pfun(nlhs, plhs, nrhs, prhs);
  }
  catch (exception const &e)
  {
    mexErrMsgTxt((string("mex_ray Error: ") + e.what()).c_str());
  }
  catch (...)
  {
    mexErrMsgTxt("mex_ray failed\n");
  }
}

///
/// eof: mex_ray.cc
///
