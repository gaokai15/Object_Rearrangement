#include <list>
#include <vector>
#include <iostream>
#include <iterator>
#include <CGAL/circulator.h>
#include <CGAL/Exact_integer.h>
#include <CGAL/Nef_polyhedron_2.h>
#include <CGAL/number_utils_classes.h>
#include <CGAL/Filtered_extended_homogeneous.h>
#include <boost/python.hpp>
#include <boost/python/list.hpp>
#include <boost/python/tuple.hpp>
#include <boost/unordered_map.hpp>
#include <boost/dynamic_bitset.hpp>
#include <boost/python/iterator.hpp>
#include <boost/container_hash/hash.hpp>
#include <boost/pending/disjoint_sets.hpp>
#include <boost/property_map/vector_property_map.hpp>

const bool DEBUG = false;

namespace py = boost::python;

typedef CGAL::Exact_integer RT;
typedef CGAL::Filtered_extended_homogeneous<RT> Extended_kernel;
typedef CGAL::Nef_polyhedron_2<Extended_kernel> NefPoly;
typedef NefPoly::Point Point;
typedef NefPoly::Line Line;
typedef NefPoly::Explorer Explorer;
typedef Explorer::Vertex_const_handle Vert;
typedef Explorer::Halfedge_around_face_const_circulator Hfc_circulator;
typedef Explorer::Isolated_vertex_const_iterator Iso_Vert_it;
typedef Explorer::Face_const_iterator Face_it;
typedef Explorer::Hole_const_iterator Hole_it;
// typedef Explorer::Halfedge_const_iterator Hedge_it;
// typedef Explorer::Vertex_const_iterator Vert_it;
typedef CGAL::Circulator_from_iterator<std::vector<Point>::iterator> Cyclic_Points;

#define EXCLUDED (NefPoly::EXCLUDED)
#define INCLUDED (NefPoly::INCLUDED)

struct XY {
	double x;
	double y;
	XY()
	{
		x = 0;
		y = 0;
	}
	XY(Point p)
	{
		double hw = (double)p.hw();
		x = (double)p.hx() / hw;
		y = (double)p.hy() / hw;
	}
	// bool operator==(XY const &p) const
	// {
	//         return x == p.x && y == p.y;
	// }
};

struct Region {
	NefPoly poly;
	static const NefPoly null;

	Region()
	{
		poly = NefPoly(NefPoly::EMPTY);
	}

	Region(const py::list &pypoly, bool open)
	{
		Point *points = new Point[len(pypoly)];
		for (int i = 0; i < len(pypoly); i++) {
			py::list xy = py::extract<py::list>(pypoly[i]);
			if (DEBUG)
				std::cerr << py::extract<double>(xy[0]) << ", " << py::extract<double>(xy[1]) << '\n';
			double x = py::extract<double>(xy[0]);
			double y = py::extract<double>(xy[1]);
			points[i] = Point(x, y);
		}
		auto boundary = open ? EXCLUDED : INCLUDED;
		poly = NefPoly(points, points + len(pypoly), boundary);
	}

	Region(const NefPoly &init_poly)
	{
		poly = init_poly;
	}

	int test();

	std::string print()
	{
		std::stringstream ss;
		ss << poly;
		return ss.str();
	}

	bool is_empty()
	{
		return poly.is_empty();
	}

	bool contains(double x, double y)
	{
		return poly.contains(poly.locate(Point(x, y)));
	}

	bool contained_in_boundary(double x, double y)
	{
		return poly.contained_in_boundary(poly.locate(Point(x, y)));
	}

	// Object_handle	ray_shoot (const Point &p, const Direction &d, Location_mode m=DEFAULT)
	// Object_handle	ray_shoot_to_boundary (const Point &p, const Direction &d, Location_mode m=DEFAULT)

	Region complement()
	{
		return Region(poly.complement());
	}

	Region interior()
	{
		return Region(poly.interior());
	}

	Region closure()
	{
		return Region(poly.closure());
	}

	Region boundary()
	{
		return Region(poly.boundary());
	}

	Region regularization()
	{
		return Region(poly.regularization());
	}

	Region intersection(const Region &r)
	{
		return Region(poly.intersection(r.poly));
	}

	Region join(const Region &r)
	{
		return Region(poly.join(r.poly));
	}

	Region difference(const Region &r)
	{
		return Region(poly.difference(r.poly));
	}

	Region symmetric_difference(const Region &r)
	{
		return Region(poly.symmetric_difference(r.poly));
	}

	bool subset(const Region &r)
	{
		return poly <= r.poly;
	}

	bool proper_subset(const Region &r)
	{
		return poly < r.poly;
	}

	bool equals(const Region &r)
	{
		return poly == r.poly;
	}

	bool not_equals(const Region &r)
	{
		return poly != r.poly;
	}

	bool superset(const Region &r)
	{
		return poly >= r.poly;
	}

	bool proper_superset(const Region &r)
	{
		return poly > r.poly;
	}

	int number_of_vertices()
	{
		return poly.explorer().number_of_vertices() - null.explorer().number_of_vertices();
	}

	int number_of_halfedges()
	{
		return poly.explorer().number_of_halfedges() - null.explorer().number_of_halfedges();
	}

	int number_of_edges()
	{
		return poly.explorer().number_of_edges() - null.explorer().number_of_edges();
	}

	int number_of_faces()
	{
		return poly.explorer().number_of_faces() - null.explorer().number_of_faces();
	}

	int number_of_face_cycles()
	{
		return poly.explorer().number_of_face_cycles() - null.explorer().number_of_face_cycles();
	}

	int number_of_connected_components()
	{
		return poly.explorer().number_of_connected_components() -
		       null.explorer().number_of_connected_components();
	}

	py::list to_list();

	bool disconnected();

	py::list get_components();

	bool operator==(Region const &o) const
	{
		return poly == o.poly;
	}
};
const NefPoly Region::null = NefPoly(NefPoly::EMPTY);

struct RHash {
	std::size_t operator()(Point const &p) const noexcept
	{
		std::size_t seed = 0;
		boost::hash_combine(seed, boost::hash_value((int)p.hx()));
		boost::hash_combine(seed, boost::hash_value((int)p.hy()));
		boost::hash_combine(seed, boost::hash_value((int)p.hw()));
		return seed;
	}

	std::size_t operator()(NefPoly const &poly) const noexcept
	{
		std::size_t seed = 0;
		Explorer expl = poly.explorer();
		Explorer::Point_const_iterator pit = expl.points_begin(), pend = expl.points_end();
		for (; pit != pend; pit++) {
			if (pit->is_standard()) {
				boost::hash_combine(seed, RHash{}(expl.point(pit)));
			}
		}
		return seed;
	}

	std::size_t operator()(Region const &r) const noexcept
	{
		return RHash{}(r.poly);
	}
};

py::list Region::to_list()
{
	// const bool DEBUG = true;
	py::list pypoly = py::list();
	if (poly.is_empty())
		return pypoly;

	Explorer expl = poly.explorer();
	Face_it fit = ++expl.faces_begin(), fend = expl.faces_end();
	for (int f = 1; fit != fend; fit++, f++) {
		if (DEBUG)
			std::cerr << "Face " << f << " (" << fit->mark() << "):\n";
		if (fit->mark() && !DEBUG)
			continue;

		Hfc_circulator fhafc = expl.face_cycle(fit);
		if (fhafc != Hfc_circulator())
			if (expl.is_frame_edge(fhafc))
				break;
	}

	Hole_it hit = expl.holes_begin(fit), hend = expl.holes_end(fit);

	if (hit == hend) {
		Iso_Vert_it ivit = expl.isolated_vertices_begin(fit), ivend = expl.isolated_vertices_end(fit);
		if (std::distance(ivit, ivend) > 1)
			if (DEBUG)
				std::cerr << "Warning: Region is disconnected!\n"
				          << "Split into components first (run get_components())\n";

		if (DEBUG) {
			std::cerr << "--- Isolated Vertex (" << ivit->mark() << ") ---\n";
			std::cerr << expl.point(ivit) << '\n';
		}
		XY p = XY(expl.point(ivit));
		py::list single = py::list();
		single.append(py::make_tuple(p.x, p.y));
		pypoly.append(single);
		return pypoly;
	}

	if (std::distance(hit, hend) > 1)
		if (DEBUG)
			std::cerr << "Warning: Region is disconnected!\n"
			          << "Split into components first (run get_components())\n";

	if (DEBUG)
		std::cerr << "--- Outer (" << hit->mark() << ") ---\n";
	Hfc_circulator ohafc(hit), odone(hit);
	std::vector<XY> opoints;
	do {
		Vert osrc = expl.source(ohafc);
		if (expl.is_standard(osrc)) {
			if (DEBUG)
				std::cerr << "\t" << expl.point(osrc) << ", " << osrc->mark() << '\n';
			opoints.push_back(XY(expl.point(osrc)));
		}
		ohafc--;
	} while (ohafc != odone);
	py::list outer = py::list();
	for (auto pit = opoints.begin(); pit != opoints.end(); pit++)
		outer.append(py::make_tuple(pit->x, pit->y));
	pypoly.append(outer);

	fit = ++expl.faces_begin();
	for (int f = 1; fit != fend; fit++, f++) {
		if (DEBUG)
			std::cerr << "Face " << f << " (" << fit->mark() << "):\n";
		if (!fit->mark() && !DEBUG)
			continue;

		Hfc_circulator fhafc = expl.face_cycle(fit);
		if (fhafc != Hfc_circulator())
			if (!expl.is_frame_edge(fhafc))
				break;
	}

	hit = expl.holes_begin(fit), hend = expl.holes_end(fit);
	for (int i = 0; hit != hend; hit++, i++) {
		if (DEBUG)
			std::cerr << "Hole " << i << " (" << hit->mark() << "):\n";
		Hfc_circulator hhafc(hit), hdone(hit);
		std::vector<XY> hpoints;
		do {
			Vert hsrc = expl.source(hhafc);
			if (expl.is_standard(hsrc)) {
				if (DEBUG)
					std::cerr << "\t" << expl.point(hsrc) << ", " << hsrc->mark() << '\n';
				hpoints.push_back(XY(expl.point(hsrc)));
			}
			hhafc--;
		} while (hhafc != hdone);
		py::list hole = py::list();
		for (auto pit = hpoints.begin(); pit != hpoints.end(); pit++)
			hole.append(py::make_tuple(pit->x, pit->y));
		pypoly.append(hole);
	}

	return pypoly;
}

typedef boost::vector_property_map<int> boost_vector;

bool Region::disconnected()
{
	if (poly.is_empty())
		return false;

	Explorer expl = poly.explorer();
	Face_it fit = ++expl.faces_begin(), fend = expl.faces_end();
	Face_it oface = fit;
	for (int f = 0; fit != fend; fit++) {
		if (DEBUG)
			std::cerr << "Face " << f << " (" << fit->mark() << "):\n";
		if (!fit->mark()) {
			Hfc_circulator fhafc = expl.face_cycle(fit);
			if (fhafc != Hfc_circulator())
				if (expl.is_frame_edge(fhafc))
					oface = fit;
		}
	}
	Iso_Vert_it ivit = expl.isolated_vertices_begin(oface), ivend = expl.isolated_vertices_end(oface);
	int c = std::distance(ivit, ivend);
	if (c > 1)
		return true;
	Hole_it hit = expl.holes_begin(oface), hend = expl.holes_end(oface);
	c += std::distance(hit, hend);
	if (c > 1)
		return true;

	boost::unordered_map<NefPoly, bool, RHash> visited;
	Hfc_circulator hafc(hit), done(hit);
	std::vector<Point> points;
	Vert tgt = expl.target(hafc);
	if (expl.is_standard(tgt)) {
		if (DEBUG)
			std::cerr << '\t' << expl.point(tgt) << ", " << tgt->mark() << '\n';
		points.push_back(expl.point(tgt));
	}
	NefPoly pvert = NefPoly(--points.end(), points.end());
	do {
		Vert src = expl.source(hafc);
		if (expl.is_standard(src)) {
			points.push_back(expl.point(src));
			NefPoly vert = NefPoly(--points.end(), points.end());
			NefPoly line = NefPoly(----points.end(), points.end()) - pvert - vert;
			pvert = vert;
			if (DEBUG)
				std::cerr << '\t' << expl.point(src) << ", " << (vert < poly) << "=" << src->mark()
				          << "\n\t\t L__ e: " << (line < poly) << "=" << hafc->mark() << '\n';
			if (!(line < poly)) {
				if (visited[line])
					return true;
				else
					visited[line] = true;
			}
			if (!(vert < poly)) {
				if (visited[vert])
					return true;
				else
					visited[vert] = true;
			}
			hafc--;
		}
	} while (hafc != done);

	// boost::unordered_map<NefPoly, bool, RHash> concentric;
	// for (fit = ++expl.faces_begin(); fit != fend; fit++, f++) {
	//         if (DEBUG)
	//                 std::cerr << "Face " << f << " (" << fit->mark() << "):\n";

	//         Hfc_circulator fhafc = expl.face_cycle(fit);
	//         if (fhafc != Hfc_circulator()) {
	//                 Hfc_circulator fdone(fhafc);
	//                 std::vector<Point> fpoints;
	//                 do {
	//                         Vert tgt = expl.target(fhafc);
	//                         if (expl.is_standard(tgt)) {
	//                                 fpoints.push_back(expl.point(tgt));
	//                                 if (DEBUG)
	//                                         std::cerr << '\t' << expl.point(tgt) << tgt->mark()
	//                                                   << "\n\t\t L__ e: " << fhafc->mark() << '\n';
	//                         } else {
	//                                 if (DEBUG)
	//                                         std::cerr << "***\tray?: " << expl.ray(tgt) << '\n';
	//                         }
	//                         fhafc++;
	//                 } while (fhafc != fdone);
	//                 if (fpoints.size() > 1) {
	//                         NefPoly face(fpoints.begin(), fpoints.end(), EXCLUDED);
	//                         if (!(face <
	//                         concentric[face] =
	//                 }
	//         }
	// }

	return false;
}

py::list Region::get_components()
{
	// const bool DEBUG = true;
	py::list pypoly = py::list();
	if (poly.is_empty())
		return pypoly;

	boost_vector rank;
	boost_vector parent;
	boost::disjoint_sets<boost_vector, boost_vector> ds(rank, parent);
	boost::unordered_map<int, NefPoly> fi2poly;
	boost::unordered_map<NefPoly, std::list<int>, RHash> poly2fis;

	Explorer expl = poly.explorer();
	Face_it fit = ++expl.faces_begin(), fend = expl.faces_end();
	Face_it oface = fit;

	if (DEBUG)
		std::cerr << "--- Faces ---\n";
	int f = 1;
	for (; fit != fend; fit++, f++) {
		ds.make_set(f);
		if (DEBUG)
			std::cerr << "Face " << f << " (" << fit->mark() << "):\n";

		Hfc_circulator fhafc = expl.face_cycle(fit);
		if (fhafc != Hfc_circulator()) {
			Hfc_circulator fdone(fhafc);
			NefPoly boundary;
			std::vector<Point> fpoints;
			Vert fsrc = expl.source(fhafc);
			NefPoly fpvert;
			if (expl.is_standard(fsrc)) {
				if (DEBUG)
					std::cerr << '\t' << expl.point(fsrc) << ", " << fsrc->mark() << '\n';
				fpoints.push_back(expl.point(fsrc));
				fpvert = NefPoly(--fpoints.end(), fpoints.end());
			} else {
				oface = fit;
				if (DEBUG)
					fpvert = NefPoly();
				else
					continue;
			}
			do {
				Vert tgt = expl.target(fhafc);
				if (expl.is_standard(tgt)) {
					fpoints.push_back(expl.point(tgt));
					NefPoly vert = NefPoly(--fpoints.end(), fpoints.end());
					NefPoly line = NefPoly(----fpoints.end(), fpoints.end()) - fpvert - vert;
					fpvert = vert;
					if (DEBUG)
						std::cerr << '\t' << expl.point(tgt) << ", " << (vert < poly) << "="
						          << tgt->mark() << "\n\t\t L__ e: " << (line < poly) << "="
						          << fhafc->mark() << '\n';
					if (line < poly) {
						boundary += line;
						BOOST_FOREACH (int i, poly2fis[line]) {
							ds.union_set(f, i);
						}
						poly2fis[line].push_back(f);
					}
					if (vert < poly) {
						boundary += vert;
						BOOST_FOREACH (int i, poly2fis[vert]) {
							ds.union_set(f, i);
						}
						poly2fis[vert].push_back(f);
					}
				} else {
					if (DEBUG)
						std::cerr << "***\tray?: " << expl.ray(tgt) << '\n';
				}
				fhafc++;
			} while (fhafc != fdone);
			if (!fit->mark())
				continue;
			NefPoly face(++fpoints.begin(), fpoints.end(), EXCLUDED);

			if (DEBUG)
				std::cerr << "Is Hole? " << poly2fis[face].size() << '\n';
			if (poly2fis[face].size())
				continue;

			NefPoly to_remove;

			Iso_Vert_it ivit = expl.isolated_vertices_begin(fit), ivend = expl.isolated_vertices_end(fit);
			for (int i = 0; ivit != ivend; ivit++, i++) {
				if (DEBUG)
					std::cerr << "\tVert " << i << ": " << expl.point(ivit) << ", " << ivit->mark()
					          << '\n';
				Point point[1] = {expl.point(ivit)};
				to_remove += NefPoly(point, point + 1);
			}

			Hole_it hit = expl.holes_begin(fit), hend = expl.holes_end(fit);
			for (int i = 0; hit != hend; hit++, i++) {
				if (DEBUG)
					std::cerr << "\tHole " << i << " (" << hit->mark() << "):\n";
				Hfc_circulator hhafc(hit), hdone(hit);
				std::vector<Point> hpoints;
				Vert tgt = expl.target(hhafc);
				if (expl.is_standard(tgt)) {
					if (DEBUG)
						std::cerr << "\t\t" << expl.point(tgt) << ", " << tgt->mark() << '\n';
					hpoints.push_back(expl.point(tgt));
				}
				NefPoly hpvert = NefPoly(--hpoints.end(), hpoints.end());
				do {
					Vert hsrc = expl.source(hhafc);
					if (expl.is_standard(hsrc)) {
						hpoints.push_back(expl.point(hsrc));
						NefPoly vert = NefPoly(--hpoints.end(), hpoints.end());
						NefPoly line =
						        NefPoly(----hpoints.end(), hpoints.end()) - hpvert - vert;
						hpvert = vert;
						if (DEBUG)
							std::cerr << "\t\t" << expl.point(hsrc) << ", " << (vert < poly)
							          << "=" << hsrc->mark()
							          << "\n\t\t\t L__ e: " << (line < poly) << "="
							          << hhafc->mark() << '\n';
						if (!(line < poly))
							to_remove += line;
						if (!(vert < poly))
							to_remove += vert;
					}
					hhafc--;
				} while (hhafc != hdone);
				NefPoly hole(++hpoints.begin(), hpoints.end(), EXCLUDED);
				assert(poly2fis[hole].empty());
				poly2fis[hole].push_back(f);
				to_remove += hole;
			}

			if (!to_remove.is_empty()) {
				face -= to_remove;
			}
			if (!boundary.is_empty()) {
				face += boundary;
			}
			fi2poly[f] = face;
		}
	}

	// fit = ++expl.faces_begin();

	if (DEBUG)
		std::cerr << "--- Isolated Vertices ---\n";
	Iso_Vert_it ivit = expl.isolated_vertices_begin(oface), ivend = expl.isolated_vertices_end(oface);
	for (int i = 0; ivit != ivend; ivit++, i++) {
		if (DEBUG)
			std::cerr << "Vert " << i << ": " << expl.point(ivit) << ", " << ivit->mark() << '\n';
		Point point[1] = {expl.point(ivit)};
		pypoly.append(Region(NefPoly(point, point + 1)));
	}

	if (DEBUG)
		std::cerr << "--- Isolated Edges ---\n";
	Hole_it hit = expl.holes_begin(oface), hend = expl.holes_end(oface);
	for (int i = 0; hit != hend; hit++, i++, f++) {
		ds.make_set(f);
		if (DEBUG)
			std::cerr << "Hole " << i << " (" << hit->mark() << ") \n";
		Hfc_circulator hhafc(hit), hdone(hit);
		std::vector<Point> opoints;
		Vert htgt = expl.target(hhafc);
		if (expl.is_standard(htgt)) {
			if (DEBUG)
				std::cerr << '\t' << expl.point(htgt) << ", " << htgt->mark() << '\n';
			opoints.push_back(expl.point(htgt));
		}
		NefPoly hpvert = NefPoly(--opoints.end(), opoints.end());
		do {
			Vert hsrc = expl.source(hhafc);
			if (expl.is_standard(hsrc)) {
				opoints.push_back(expl.point(hsrc));
				NefPoly vert = NefPoly(--opoints.end(), opoints.end());
				NefPoly line = NefPoly(----opoints.end(), opoints.end()) - hpvert - vert;
				hpvert = vert;
				if (DEBUG)
					std::cerr << '\t' << expl.point(hsrc) << ", " << (vert < poly) << "="
					          << hsrc->mark() << "\n\t\t L__ e: " << (line < poly) << "="
					          << hhafc->mark() << '\n';
				if (line < poly) {
					BOOST_FOREACH (int i, poly2fis[line]) {
						ds.union_set(f, i);
					}
					poly2fis[line].push_back(f);
					// if (fi2poly.find(f) == fi2poly.end())
					//         fi2poly[f] = line;
					// else
					fi2poly[f] += line;
				}
				if (vert < poly) {
					BOOST_FOREACH (int i, poly2fis[vert]) {
						ds.union_set(f, i);
					}
					poly2fis[vert].push_back(f);
					// if (fi2poly.find(f) == fi2poly.end())
					//         fi2poly[f] = vert;
					// else
					fi2poly[f] += vert;
				} else {
					f++;
					ds.make_set(f);
				}
			}
			hhafc--;
		} while (hhafc != hdone);
		// NefPoly outer(++opoints.begin(), opoints.end(), EXCLUDED);
		// poly2fis[outer].size();
	}

	// Consolidate Faces / Edges
	for (auto fip = fi2poly.begin(); fip != fi2poly.end(); fip++) {
		size_t fi = fip->first;
		size_t pi = ds.find_set(fi);
		if (DEBUG)
			std::cerr << "fi->pi: " << fi << "->" << pi << '\n' << fip->second << '\n';
		if (fi != pi)
			fi2poly[pi] += fip->second;
	}
	for (auto fip = fi2poly.begin(); fip != fi2poly.end(); fip++) {
		size_t fi = fip->first;
		size_t pi = ds.find_set(fi);
		if (fi == pi)
			pypoly.append(Region(fip->second));
	}

	return pypoly;
}

BOOST_PYTHON_MODULE(region)
{
	py::class_<Region>("region", py::init<>())
	        .def(py::init<py::list &, bool>())
	        .def("__str__", &Region::print)
	        .def("__mul__", &Region::intersection)
	        .def("__add__", &Region::join)
	        .def("__sub__", &Region::difference)
	        .def("__and__", &Region::intersection)
	        .def("__or__", &Region::join)
	        .def("__xor__", &Region::symmetric_difference)
	        .def("__invert__", &Region::complement)
	        .def("__le__", &Region::subset)
	        .def("__lt__", &Region::proper_subset)
	        .def("__eq__", &Region::equals)
	        .def("__ne", &Region::not_equals)
	        .def("__ge__", &Region::superset)
	        .def("__gt__", &Region::proper_superset)
	        .def("is_empty", &Region::is_empty)
	        .def("contains", &Region::contains)
	        .def("contained_in_boundary", &Region::contained_in_boundary)
	        .def("complement", &Region::complement)
	        .def("interior", &Region::interior)
	        .def("closure", &Region::closure)
	        .def("boundary", &Region::boundary)
	        .def("regularization", &Region::regularization)
	        .def("intersection", &Region::intersection)
	        .def("join", &Region::join)
	        .def("difference", &Region::difference)
	        .def("symmetric_difference", &Region::symmetric_difference)
	        .def("num_vertices", &Region::number_of_vertices)
	        .def("num_halfedges", &Region::number_of_halfedges)
	        .def("num_edges", &Region::number_of_edges)
	        .def("num_faces", &Region::number_of_faces)
	        .def("num_face_cyles", &Region::number_of_face_cycles)
	        .def("num_connected_components", &Region::number_of_connected_components)
	        .def("get_components", &Region::get_components)
	        .def("disconnected", &Region::disconnected)
	        .def("to_list", &Region::to_list);
	// py::def(
	//         "test", +[]() {
	//                 Point ps1[4] = {Point(50, 50), Point(950, 50), Point(950, 950), Point(50, 950)};
	//                 Point ps2[12] = {Point(784, 578), Point(798, 528), Point(834, 492), Point(884, 478),
	//                                  Point(934, 492), Point(970, 528), Point(984, 578), Point(970, 626),
	//                                  Point(934, 664), Point(884, 678), Point(836, 664), Point(798, 626)};
	//                 NefPoly p1(ps1, ps1 + 4);
	//                 NefPoly p2(ps2, ps2 + 12);
	//                 NefPoly p3 = p1.intersection(p2);

	//                 Region r(p3);
	//                 return r.get_components();
	//         });
}
