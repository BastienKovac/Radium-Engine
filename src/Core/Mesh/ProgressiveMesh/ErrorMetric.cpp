#include <Core/Mesh/ProgressiveMesh/ErrorMetric.hpp>

#include <Core/Geometry/Triangle/TriangleOperation.hpp>

#include <Core/Mesh/DCEL/Iterator/Edge/EFIterator.hpp>
#include <Core/Mesh/DCEL/Iterator/Face/FFIterator.hpp>
#include <Core/Mesh/DCEL/Iterator/Vertex/VVIterator.hpp>
#include <Core/Mesh/DCEL/Iterator/Vertex/VFIterator.hpp>
#include <Core/Mesh/DCEL/Operations/VertexOperation.hpp>

#include <Core/Log/Log.hpp>

namespace Ra
{
    namespace Core
    {

        //---------------------------------------------------

        QuadricErrorMetric::QuadricErrorMetric()
        {
        }

        QuadricErrorMetric::QuadricErrorMetric(Scalar scale)
        {
        }

        Scalar QuadricErrorMetric::computeGeometricError(const Primitive& q, const Primitive::Vector& p)
        {
            // Computing geometric error
            // v^T A v + 2 * b^T v + c
            Eigen::Matrix<Scalar, 1, 3> row_p = p.transpose();
            Eigen::Matrix<Scalar, 1, 3> row_b = q.getB().transpose();
            Scalar error_a = row_p * q.getA() * p;
            Scalar error_b = 2.0 * row_b * p;
            Scalar error_c = q.getC();
            return (error_a + error_b + error_c);
        }

        Scalar QuadricErrorMetric::computeError(HalfEdge_ptr he, std::vector<Primitive>& primitives_v, Vector3& pResult, Primitive& q)
        {
            Vertex_ptr vs = he->V();
            Vertex_ptr vt = he->Next()->V();
            q = primitives_v[vs->idx] * 0.5 + primitives_v[vt->idx] * 0.5;

            Scalar error;

            // on cherche v_result
            // A v_result = -b		avec A = nn^T
            //							 b = dn
            Primitive::Matrix AInverse = q.getA().inverse();
            Primitive::Vector result;

            Scalar det = q.getA().determinant();
            if (det > 0.0001)
            {
                result = -AInverse * q.getB();
                error = computeGeometricError(q, result);
            }
            else //matrix non inversible
            {
                Primitive::Vector p1  = vs->P();
                Primitive::Vector p2  = vt->P();
                Primitive::Vector p12 = (p1 + p2) / 2.0;

                Scalar p1_error     = computeGeometricError(q, p1);
                Scalar p2_error     = computeGeometricError(q, p2);
                Scalar p12_error    = computeGeometricError(q, p12);

                error = p1_error;
                Primitive::Vector p = p1;
                if (p2_error < error && p12_error > p2_error)
                {
                    p = p2;
                    result = p;
                    error = p2_error;
                }
                else if (p12_error < error && p2_error > p12_error)
                {
                    p = p12;
                    result = p;
                    error = p12_error;
                }
                else
                {
                    result = p;
                }
            }
            pResult = Vector3(result.x(), result.y(), result.z());
            return error;
        }

        QuadricErrorMetric::Primitive QuadricErrorMetric::combine(const std::vector<Primitive>& p, const std::vector<Scalar>& weights, Scalar normalizing_weight_factor)
        {
            Primitive sum;

            for (unsigned int i = 0; i < p.size(); i++)
            {
                sum += p[i] * weights[i];
            }

            return sum;
        }

        void QuadricErrorMetric::generateVertexPrimitive(Primitive &q, Vertex_ptr v, Scalar weight, int ringSize)
        {
            VFIterator vfIt = VFIterator(v);
            FaceList adjFaces = vfIt.list();

            for (unsigned int i = 0; i < adjFaces.size(); i++)
            {
                Face_ptr fi = adjFaces[i];
                Scalar weight = getWedgeAngle(fi, v);
                Primitive qi;
                generateFacePrimitive(qi, fi, weight, ringSize);
                q += qi * weight;
            }
        }

        void QuadricErrorMetric::generateFacePrimitive(Primitive &q, Face_ptr f, Scalar weight, int ringSize)
        {
            Vertex_ptr v0 = f->HE()->V();
            Vertex_ptr v1 = f->HE()->Next()->V();
            Vertex_ptr v2 = f->HE()->Next()->Next()->V();

            Primitive::Vector n = Geometry::triangleNormal(v0->P(), v1->P(), v2->P());
            q = Primitive(n, -n.dot(v0->P()));
        }

        Scalar QuadricErrorMetric::getWedgeAngle(Face_ptr face, Vertex_ptr v)
        {
            Scalar wedgeAngle;

            HalfEdge_ptr he = face->HE();
            for (int i = 0; i < 3; i++)
            {
                if (he->V() == v)
                {
                    Vector3 v0 = he->Next()->V()->P() - he->V()->P();
                    Vector3 v1 = he->Prev()->V()->P() - he->V()->P();
                    v0.normalize();
                    v1.normalize();
                    wedgeAngle = std::acos(v0.dot(v1));
                    break;
                }
                he = he->Next();
            }
            return wedgeAngle;
        }

        //---------------------------------------------------

        SimpleAPSSErrorMetric::SimpleAPSSErrorMetric()
        {
        }

        Scalar SimpleAPSSErrorMetric::computeEdgeMinErrorOnEdge(HalfEdge_ptr he, std::vector<Primitive>& v_primitives, Vector3& pResult, Primitive &q)
        {
            Vertex_ptr vs = he->V();
            Vertex_ptr vt = he->Next()->V();
            Vector3 vsPos = vs->P();
            Vector3 vtPos = vt->P();
            Primitive q1 = v_primitives[vs->idx];
            Primitive q2 = v_primitives[vt->idx];

            // computing alpha
            // sampling along the segments in 3D (TODO, sampling along uv)
            Scalar dt = 0.01;
            Scalar min_error = std::numeric_limits<double>::max();
            Scalar min_alpha = 0.0;

            std::vector<Scalar> errors;
            errors.reserve(100);


            if (q1.hasSameParameter(q2) && std::abs((q1.m_ul).dot((vtPos - vsPos).normalized())) < 0.00001)
            {
                // both spheres are planes
                min_alpha = 0.5;
                min_error = q1.potential(vsPos);
            }
            else
            {

                for (Scalar alpha = 0.0; alpha <= 1.0; alpha += dt) // we go through the edge

                {
                    //Scalar alpha = 0.5;
                    Vector3 valpha = vsPos + alpha * (vtPos - vsPos);
                    Primitive salpha;
                    salpha.combine(q1, q2, alpha);


//                    // find adjacent vertices to vs and vt
//                    VVIterator vsvIt = VVIterator(vs);
//                    VertexList adjVerticesVs = vsvIt.list();
//                    VVIterator vtvIt = VVIterator(vt);
//                    VertexList adjVerticesVt = vtvIt.list();

//                    Scalar error = 0.0;
//                    valpha = salpha.project(valpha);
//                    Scalar sum_edge_norm = 0.0;
//                    for (unsigned int i = 0; i < adjVerticesVs.size(); i++)
//                    {
//                        if (adjVerticesVs[i]->idx == vt->idx) continue;
//                        sum_edge_norm += (adjVerticesVs[i]->P() - valpha).norm();
//                        Scalar dist_seg_sphere = salpha.AlgebraicSphere::distanceSegSphere(adjVerticesVs[i]->P(), valpha);
//                        //Scalar dist_seg_sphere = q1.AlgebraicSphere::distanceSegSphere(adjVerticesVs[i]->P(), valpha);
//                        //Scalar dist_seg_sphere = std::abs(q1.AlgebraicSphere::distanceSegSphere(adjVerticesVs[i]->P(), valpha));
//                        error += dist_seg_sphere;
//                    }
//                    for (unsigned int i = 0; i < adjVerticesVt.size(); i++)
//                    {
//                        if (adjVerticesVt[i]->idx == vs->idx) continue;
//                        sum_edge_norm += (adjVerticesVt[i]->P() - valpha).norm();
//                        Scalar dist_seg_sphere = salpha.AlgebraicSphere::distanceSegSphere(valpha, adjVerticesVt[i]->P());
//                        //Scalar dist_seg_sphere = q2.AlgebraicSphere::distanceSegSphere(valpha, adjVerticesVt[i]->P());
//                        //Scalar dist_seg_sphere = std::abs(q2.AlgebraicSphere::distanceSegSphere(valpha, adjVerticesVt[i]->P()));
//                        error += dist_seg_sphere;
//                    }
//                    error /= sum_edge_norm;

                    // find adjacent faces to vs and vt
                    EFIterator hefIt = EFIterator(he);
                    FaceList adjFaces = hefIt.list();

                    Scalar error = 0.0;
                    valpha = salpha.project(valpha);
                    Scalar sum_face_area = 0.0;
                    for (unsigned int i = 0; i < adjFaces.size(); i++)
                    {
                        if (adjFaces[i]->idx == he->F()->idx) continue;
                        if (adjFaces[i]->idx == he->Twin()->F()->idx) continue;
                        Vector3 vert_pos[3];
                        HalfEdge_ptr he_through = adjFaces[i]->HE();
                        for (uint j = 0; j < 3; j++)
                        {
                            if (he_through->V()->idx == vs->idx || he_through->V()->idx == vt->idx)
                            {
                                vert_pos[j] = valpha;
                            }
                            else
                            {
                                vert_pos[j] = he_through->V()->P();
                            }
                            he_through = he_through->Next();
                        }
                        sum_face_area += Geometry::triangleArea(vert_pos[0], vert_pos[1], vert_pos[2]);
                        Scalar dist_face_sphere = std::abs(salpha.AlgebraicSphere::distanceFaceSphere(vert_pos[0], vert_pos[1], vert_pos[2]));
                        //dist_face_sphere += std::abs(salpha.primitiveGradient(valpha).dot(vtPos - vsPos));
                        //Scalar dist_face_sphere = std::abs(salpha.AlgebraicSphere::distanceFaceSphere(vert_pos[0], vert_pos[1], vert_pos[2]));
                        //dist_face_sphere += std::abs(salpha.primitiveGradient(valpha).dot((vtPos - vsPos).normalized()));
                        error += dist_face_sphere;
                    }
                    //error /= sum_face_area;
                    //error /= adjFaces.size();

                    if (std::abs(error) < min_error)
                    {
                        min_error = std::abs(error);
                        min_alpha = alpha;
                    }
                    //errors.push_back(std::abs(error));
                }
//                file << q1.m_uc << " " << q1.m_ul.transpose() << " " << q1.m_uq << " " << q1.basisCenter().transpose() << "\n";
//                file << q2.m_uc << " " << q2.m_ul.transpose() << " " << q2.m_uq << " " << q2.basisCenter().transpose() << "\n";
//                for (int i = 0; i < 100; i++)
//                {
//                    file << errors[i] << " ";
//                }
//                file << "\n";
            }

            q.combine(q1, q2, min_alpha);
            pResult = vsPos + min_alpha * (vtPos - vsPos);
            //pResult = q.project(pResult);
            return min_error;
        }

        Scalar SimpleAPSSErrorMetric::computeEdgeMinErrorOnFace(HalfEdge_ptr he, std::vector<Primitive>& v_primitives, Vector3& pResult, Primitive &q)
        {
            Vertex_ptr vs = he->V();
            Vertex_ptr vt = he->Next()->V();

            Scalar min_error = std::numeric_limits<double>::max();
            min_error = computeEdgeMinErrorOnEdge(he, v_primitives, pResult, q);
            Scalar min_gamma1 = 0.0;
            Scalar min_gamma2 = 0.0;

            //EFIterator hefIt = EFIterator(he);
            //FaceList adjFaces = hefIt.list();
            FaceList adjFaces;
            adjFaces.push_back(he->F());
            adjFaces.push_back(he->Twin()->F());

            Scalar dt = 0.1;
            for (uint fi = 0; fi < adjFaces.size(); ++fi)
            {
                Face_ptr f = adjFaces[fi];
                Vertex_ptr v0 = f->HE()->V();
                Vertex_ptr v1 = f->HE()->Next()->V();
                Vertex_ptr v2 = f->HE()->Next()->Next()->V();
                Primitive q0 = v_primitives[v0->idx];
                Primitive q1 = v_primitives[v1->idx];
                Primitive q2 = v_primitives[v2->idx];
                if (q0.hasSameParameter(q1) && q1.hasSameParameter(q2) && std::abs((q0.m_ul).dot((vt->P() - vs->P()).normalized())) < 0.00001)
                {
                    // all spheres are planes
                    min_gamma1 = (1.0 / 3.0);
                    min_gamma2 = (1.0 / 3.0);
                    min_error = q0.potential(vs->P());
                }
                else if (q0.hasSameParameter(q1) && std::abs((q0.m_ul).dot((vt->P() - vs->P()).normalized())) < 0.00001)
                {
                    min_gamma1 = 0.5;
                    min_gamma2 = 0.5;
                    min_error = q0.potential(vs->P());
                }
                else if (q1.hasSameParameter(q2) && std::abs((q1.m_ul).dot((vt->P() - vs->P()).normalized())) < 0.00001)
                {
                    min_gamma1 = 0.0;
                    min_gamma2 = 0.5;
                    min_error = q1.potential(vs->P());
                }
                else if (q2.hasSameParameter(q0) && std::abs((q2.m_ul).dot((vt->P() - vs->P()).normalized())) < 0.00001)
                {
                    min_gamma1 = 0.5;
                    min_gamma2 = 0.0;
                    min_error = q2.potential(vs->P());
                }
                else
                {

                    for (Scalar gamma2 = 0.0; gamma2 <= 1.0; gamma2 += dt) // we go through the face
                    {
                        for (Scalar gamma1 = 0.0; gamma1 <= 1.0 - gamma2; gamma1 += dt)
                        {
                            Vector3 vgamma = gamma1 * v0->P() + gamma2 * v1->P() + (1.0 - gamma1 - gamma2) * v2->P();
                            Primitive salpha;
                            salpha.combine(q0, q1, q2, gamma1, gamma2);

                            Scalar error = 0.0;
                            vgamma = salpha.project(vgamma);
                            Scalar sum_face_area = 0.0;
                            bool consistent_normal = true;
                            for (uint i = 0; i < adjFaces.size(); i++)
                            {
                                if (adjFaces[i]->idx == he->F()->idx) continue;
                                if (adjFaces[i]->idx == he->Twin()->F()->idx) continue;
                                Vector3 vert_pos[3];
                                HalfEdge_ptr he_through = adjFaces[i]->HE();
                                for (uint j = 0; j < 3; j++)
                                {
                                    if (he_through->V()->idx == vs->idx || he_through->V()->idx == vt->idx)
                                    {
                                        vert_pos[j] = vgamma;
                                    }
                                    else
                                    {
                                        vert_pos[j] = he_through->V()->P();
                                    }
                                    he_through = he_through->Next();
                                }
                                sum_face_area += Geometry::triangleArea(vert_pos[0], vert_pos[1], vert_pos[2]);
                                Vector3 normal = Geometry::triangleNormal(vert_pos[0], vert_pos[1], vert_pos[2]);
                                consistent_normal = (normal.dot(salpha.m_ul)) < 0.0001;
                                if (!consistent_normal)
                                    break;
                                Scalar dist_face_sphere = salpha.AlgebraicSphere::distanceFaceSphere(vert_pos[0], vert_pos[1], vert_pos[2]);
                                error += dist_face_sphere;
                            }
                            error /= sum_face_area;

                            if (consistent_normal && (std::abs(error) < min_error))
                            {
                                min_error = std::abs(error);
                                pResult = vgamma;
                                q = salpha;
                                min_gamma1 = gamma1;
                                min_gamma2 = gamma2;
                            }
                        }
                    }
                }
            }
            pResult = q.project(pResult);
            return min_error;
        }

        Scalar SimpleAPSSErrorMetric::computeError(HalfEdge_ptr he, std::vector<Primitive>& v_primitives, Vector3& pResult, Primitive &q)
        {
            Scalar min_error = computeEdgeMinErrorOnEdge(he, v_primitives, pResult, q);
            //Scalar min_error = computeEdgeMinErrorOnFace(he, v_primitives, pResult, q);
            return min_error;
        }


        //------------------------------------------------------------------------------------------------
        // RIMLS : Feature Preserving Point Set Surfaces based on Non-Linear Kernel Regression
        //------------------------------------------------------------------------------------------------

        Scalar phi(Scalar x, Scalar hi)
        {
            return std::pow(1.0 - (x / (hi * hi)), 4.0);
        }

        Scalar dphi(Scalar x, Scalar hi)
        {
            return (-4.0 / (hi * hi)) * std::pow(1.0 - (x / (hi * hi)), 3.0);
        }

        bool convergence(Scalar alpha, Scalar sumA, Scalar alpha_b, Scalar sumA_b)
        {
            Scalar t = 0.0001;
            return std::abs((alpha / sumA) - (alpha_b / sumA_b)) < t;
        }

        void SimpleAPSSErrorMetric::generateRIMLSVertexPrimitive(Primitive &q, Vertex_ptr v, int ringSize)
        {
            Vector3 x = v->P();

            Fit1 fit;

            VVIterator vvIt = VVIterator(v);
            std::set<Vertex_ptr, VVIterator::compareVertexPtr> adjVerticesSet;
            vvIt.nRing(ringSize, adjVerticesSet);
            std::set<Vertex_ptr, VVIterator::compareVertexPtr>::iterator it;

            Scalar f, fx, w;
            Scalar alpha, alpha_b, sumA, sumA_b;
            Vector3 grad_w, grad_f, p, px, n;
            Scalar sigma_r, sigma_n;
            Scalar sumW, sumF;
            Vector3 sumGw, sumGf, sumN;
            Scalar threshold, hi;

            // Value choices
            // TODO
            sigma_r = 0.5;
            sigma_n = 0.5; // typical choices range from 0.5 to 1.5
            hi = 2.0 * (x - v->HE()->Twin()->V()->P()).norm();  // 1.4 to 4 times the local point spacing
                                                                // TODO
            threshold = 0.001; //verify
            int max_iters = 20;


            // projection of v onto the underlying RIMLS surface using
            // a steepest gradient descent strategy
            do
            {
                f = 0.0;
                grad_f = Vector3::Zero();
                int i = 0;
                bool converge = false;

                do
                {
                    fit.init(x);
                    uint nb_neighbors = 0;

                    sumA = sumF = sumW = 0.0;
                    sumGw = sumGf = sumN = Vector3::Zero();

                    for (it = adjVerticesSet.begin(); it != adjVerticesSet.end(); ++it)
                    {
                        sumA_b = sumA;
                        alpha_b = alpha;
                        p = (*it)->P();
                        n = DcelOperations::vertexNormal((*it));
                        px = x - p;
                        fx = px.dot(n);
                        if (i > 0)
                        {
                            alpha = std::exp(-((fx - f)/sigma_r)*((fx - f)/sigma_r)) *
                                    std::exp(-((n - grad_f).norm()/sigma_n)*((n - grad_f).norm()/sigma_n));
                        }
                        else
                        {
                            alpha = 1.0;
                        }
                        sumA += alpha;

                        w = alpha * phi(px.norm() * px.norm(), hi);
                        grad_w = alpha * 2.0 * px * dphi(px.norm() * px.norm(), hi);

                        sumW    += w;
                        sumGw   += grad_w;
                        sumF    += w * fx;
                        sumGf   += grad_w * fx;
                        sumN    += w * n;

                        GrenaillePoint gpi(p, n);
                        if (fit.addNeighbor(gpi, w))
                        {
                            nb_neighbors++;
                        }
                    }
                    f = sumF / sumW;
                    grad_f = (sumGf - f * sumGw + sumN) / sumW;
                    converge = convergence(alpha, sumA, alpha_b, sumA_b);
//                    if (i >= max_iters - 1)
//                    {
//                        LOG(logINFO) << "MAX ITERS, conv = " << std::abs((alpha / sumA) - (alpha_b / sumA_b));
//                    }
//                    else if (converge)
//                    {
//                        LOG(logINFO) << "CONV, max_iters = " << i;
//                    }
                } while ((++i < max_iters) && !converge);
                // TODO verify convergence see paper
                x = x - f * grad_f;
            } while ((f * grad_f).norm() > threshold);

            fit.finalize();
            fit.applyPrattNorm();

            if (fit.getCurrentState() != UNDEFINED)
            {
                q = fit;
                CORE_ASSERT(!std::isnan(q.m_uc), "PRIMITIVE NAN, NOT OK");
            }
            else
            {
                CORE_ASSERT(true, "PRIMITIVE NOT OK : APSS FIT IS NOT STABLE");
            }
        }

        //-----------------------------------------------------------------------------------------
        void SimpleAPSSErrorMetric::generateVertexPrimitive(Primitive &q, Vertex_ptr v, Scalar weight, int ringSize)
        {
            Vector3 p = v->P();
            Vector3 new_p = p;
            Vector3 nei = Vector3::Zero();

            Fit1 fit;
            fit.setWeightFunc(WeightFunc(weight)); // TODO weight func

            Scalar error;
            VVIterator vvIt = VVIterator(v);
            std::set<Vertex_ptr, VVIterator::compareVertexPtr> adjVerticesSet;
            vvIt.nRing(ringSize, adjVerticesSet);
            unsigned int nb_of_loop = 0;
            do {
                fit.init(new_p);
                std::set<Vertex_ptr, VVIterator::compareVertexPtr>::iterator it;
                unsigned int nb_neighbors = 0;
                for (it = adjVerticesSet.begin(); it != adjVerticesSet.end(); ++it)
                {
                    Vertex_ptr vi = *it;
                    nei = vi->P();
                    Vector3 n = DcelOperations::vertexNormal(vi);
                    GrenaillePoint gpi(nei, n);
                    if (fit.addNeighbor(gpi))
                    {
                        nb_neighbors++;
                    }
                }

                CORE_ASSERT(nb_neighbors >= 3, "PRIMITIVE COMPUTING DO NOT HAVE ENOUGH NEIGHBORS");

                fit.finalize();
                fit.applyPrattNorm();
                new_p = fit.project(p);
                error = (new_p-p).norm();
                p = new_p;
                nb_of_loop++;
                if (nb_of_loop > 100)
                    break;
            } while (error > 0.0001); // TODO threshold

            if (fit.getCurrentState() != UNDEFINED)
            {
                q = fit;
                CORE_ASSERT(!std::isnan(q.m_uc), "PRIMITIVE NAN, NOT OK");
            }
            else
            {
                CORE_ASSERT(true, "PRIMITIVE NOT OK : APSS FIT IS NOT STABLE");
            }
        }

        void SimpleAPSSErrorMetric::generateFacePrimitive(Primitive &q, Face_ptr f, Scalar weight, int ringSize)
        {
            Vertex_ptr v0 = f->HE()->V();
            Vertex_ptr v1 = f->HE()->Next()->V();
            Vertex_ptr v2 = f->HE()->Next()->Next()->V();
            Vector3 p = (v0->P() + v1->P() + v2->P()) / 3.0;
            Vector3 new_p = p;

            Fit1 fit;
            fit.setWeightFunc(WeightFunc(weight)); // TODO weight func

            Scalar error;
            FFIterator ffIt = FFIterator(f);
            std::set<Face_ptr, FFIterator::compareFacePtr> adjFacesSet;
            ffIt.nRing(ringSize, adjFacesSet); // TODO N-ring
            //ffIt.nRing(1, adjFacesSet); // TODO N-ring
            do {
                fit.init(new_p);
                std::set<Face_ptr, FFIterator::compareFacePtr>::iterator it;
                for (it = adjFacesSet.begin(); it != adjFacesSet.end(); ++it)
                {
                    Face_ptr fi = *it;
                    v0 = fi->HE()->V();
                    v1 = fi->HE()->Next()->V();
                    v2 = fi->HE()->Next()->Next()->V();
                    p = (v0->P() + v1->P() + v2->P()) / 3.0;
                    Vector3 n = Geometry::triangleNormal(v0->P(), v1->P(), v2->P());
                    GrenaillePoint gpi(p, n);
                    fit.addNeighbor(gpi);
                }

                fit.finalize();
                fit.applyPrattNorm();
                new_p = fit.project(p);
                error = (new_p-p).norm();
                p = new_p;
            } while (error > 0.01); // TODO threshold

            if (fit.getCurrentState() != UNDEFINED)
            {
                q = fit;
                CORE_ASSERT(!std::isnan(q.m_uc), "PRIMITIVE NAN, NOT OK");
            }
            else
            {
                CORE_ASSERT(true, "PRIMITIVE NOT OK : APSS FIT IS NOT STABLE");
            }
        }

    }
}

