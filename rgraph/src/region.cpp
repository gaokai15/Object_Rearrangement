// BEGIN HACK
#include <CGAL/IO/io_impl.h>
#include <CGAL/assertions_impl.h>
// END HACK
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

#define DEBUG (true)

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

// struct XY {
//         int x;
//         int y;
//         XY()
//         {
//                 x = 0;
//                 y = 0;
//         }
//         XY(Point p)
//         {
//                 x = (int)p.hx();
//                 y = (int)p.hy();
//         }
//         bool operator==(XY const &p) const
//         {
//                 return x == p.x && y == p.y;
//         }
// };

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
				std::cerr << py::extract<int>(xy[0]) << ", " << py::extract<int>(xy[1]) << '\n';
			int x = py::extract<int>(xy[0]);
			int y = py::extract<int>(xy[1]);
			points[i] = Point(x, y);
		}
		auto boundary = open ? EXCLUDED : INCLUDED;
		poly = NefPoly(points, points + len(pypoly), boundary);
	}

	Region(const NefPoly &init_poly)
	{
		poly = init_poly;
	}

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

	bool contains(int x, int y)
	{
		return poly.contains(poly.locate(Point(x, y)));
	}

	bool contained_in_boundary(int x, int y)
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
		// if (poly.is_empty())
		//         return 0;
		// Explorer expl = poly.explorer();
		// Face_it fit = ++expl.faces_begin();
		// Hole_it hit = expl.holes_begin(fit), end = expl.holes_end(fit);
		// return std::distance(hit, end);
	}

	py::list to_list();

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
	// if (DEBUG)
	//         std::cerr << poly;
	py::list pypoly = py::list();
	if (poly.is_empty())
		return pypoly;

	Explorer expl = poly.explorer();
	Face_it fit = ++expl.faces_begin(); //, fend = expl.faces_end();
	Hole_it hit = expl.holes_begin(fit), hend = expl.holes_end(fit);
	if (std::distance(hit, hend) > 1)
		if (DEBUG)
			std::cerr << "Warning: Region is disconnected!\n"
			          << "Split into components first (run get_components())\n";

	if (DEBUG)
		std::cerr << "--- Outer (" << hit->mark() << ") ---\n";
	Hfc_circulator fhafc(hit), fdone(hit);
	std::vector<Point> opoints;
	do {
		Vert osrc = expl.source(fhafc);
		if (expl.is_standard(osrc)) {
			if (DEBUG)
				std::cerr << "\t" << expl.point(osrc) << ", " << osrc->mark() << '\n';
			opoints.push_back(expl.point(osrc));
		}
		fhafc--;
	} while (fhafc != fdone);
	py::list outer = py::list();
	// outer.append(hit->mark());
	for (auto pit = opoints.begin(); pit != opoints.end(); pit++)
		outer.append(py::make_tuple((int)pit->hx(), (int)pit->hy()));
	pypoly.append(outer);

	fit++;
	hit = expl.holes_begin(fit), hend = expl.holes_end(fit);
	for (int i = 0; hit != hend; hit++, i++) {
		if (DEBUG)
			std::cerr << "Hole " << i << " (" << hit->mark() << "):\n";
		Hfc_circulator hhafc(hit), hdone(hit);
		std::vector<Point> hpoints;
		do {
			Vert hsrc = expl.source(hhafc);
			if (expl.is_standard(hsrc)) {
				if (DEBUG)
					std::cerr << "\t" << expl.point(hsrc) << ", " << hsrc->mark() << '\n';
				hpoints.push_back(expl.point(hsrc));
			}
			hhafc--;
		} while (hhafc != hdone);
		py::list hole = py::list();
		// hole.append(hit->mark());
		for (auto pit = hpoints.begin(); pit != hpoints.end(); pit++)
			hole.append(py::make_tuple((int)pit->hx(), (int)pit->hy()));
		pypoly.append(hole);
	}

	return pypoly;
}

// #define FOR_BITS(i, bs) for (size_t i = bs.find_first(); i != boost::dynamic_bitset<>::npos; i = bs.find_next(i))
// #define INIT_BITS(n, bs) bs.resize(std::max(bs.size(), (size_t)n))
typedef boost::vector_property_map<int> boost_vector;

py::list Region::get_components()
{
	py::list pypoly = py::list();
	if (poly.is_empty())
		return pypoly;

	// boost::vector_property_map<int> rank(1000);
	// boost::vector_property_map<int> parent(1000);
	// boost::disjoint_sets<int *, int *> ds(&rank[0], &parent[0]);
	boost_vector rank;
	boost_vector parent;
	boost::disjoint_sets<boost_vector, boost_vector> ds(rank, parent);
	boost::unordered_map<int, NefPoly> fi2poly;
	// boost::unordered_map<NefPoly, boost::dynamic_bitset<>, RHash> poly2fis;
	boost::unordered_map<NefPoly, std::list<int>, RHash> poly2fis;
	// boost::unordered_map<NefPoly, int, RHash> poly2face;

	Explorer expl = poly.explorer();
	Face_it fit = ++++expl.faces_begin(), fend = expl.faces_end();

	// NefPoly all_faces;
	if (DEBUG)
		std::cerr << "--- Faces ---\n";
	int f = 1;
	for (; fit != fend; fit++, f++) {
		ds.make_set(f);
		if (DEBUG)
			std::cerr << "Face " << f << " (" << fit->mark() << "):\n";
		if (!fit->mark() && !DEBUG)
			continue;

		Hfc_circulator fhafc = expl.face_cycle(fit);
		if (fhafc != Hfc_circulator()) {
			Hfc_circulator fdone(fhafc);
			NefPoly boundary;
			std::vector<Point> fpoints;
			Vert fsrc = expl.source(fhafc);
			if (expl.is_standard(fsrc)) {
				if (DEBUG)
					std::cerr << '\t' << expl.point(fsrc) << ", " << fsrc->mark() << '\n';
				fpoints.push_back(expl.point(fsrc));
			}
			NefPoly fpvert = NefPoly(--fpoints.end(), fpoints.end());
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
						// INIT_BITS(f, poly2fis[line]);
						// FOR_BITS(i, poly2fis[line])
						// {
						//         ds.union_set(f, i);
						// }
						// poly2fis[line][f] = 1;
						BOOST_FOREACH (int i, poly2fis[line]) {
							ds.union_set(f, i);
						}
						poly2fis[line].push_back(f);
					}
					if (vert < poly) {
						boundary += vert;
						// INIT_BITS(f, poly2fis[vert]);
						// FOR_BITS(i, poly2fis[vert])
						// {
						//         ds.union_set(f, i);
						// }
						// poly2fis[vert][f] = 1;
						BOOST_FOREACH (int i, poly2fis[vert]) {
							ds.union_set(f, i);
						}
						poly2fis[vert].push_back(f);
					}
				}
				fhafc++;
			} while (fhafc != fdone);
			NefPoly face(++fpoints.begin(), fpoints.end(), EXCLUDED);

			if (DEBUG)
				std::cerr << "Is Hole? " << poly2fis[face].size() << '\n';
			if (poly2fis[face].size())
				continue;
			if (!fit->mark())
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
				// INIT_BITS(f, poly2fis[hole]);
				// poly2fis[hole][f] = 1;
				poly2fis[hole].push_back(f);
				to_remove += hole;
			}

			if (!to_remove.is_empty()) {
				face -= to_remove;
			}
			if (!boundary.is_empty()) {
				face += boundary;
			}
			// all_faces += face;
			fi2poly[f] = face;
		}
	}

	fit = ++expl.faces_begin();

	if (DEBUG)
		std::cerr << "--- Isolated Vertices ---\n";
	Iso_Vert_it ivit = expl.isolated_vertices_begin(fit), ivend = expl.isolated_vertices_end(fit);
	for (int i = 0; ivit != ivend; ivit++, i++) {
		if (DEBUG)
			std::cerr << "Vert " << i << ": " << expl.point(ivit) << ", " << ivit->mark() << '\n';
		Point point[1] = {expl.point(ivit)};
		pypoly.append(Region(NefPoly(point, point + 1)));
	}

	if (DEBUG)
		std::cerr << "--- Isolated Edges ---\n";
	Hole_it hit = expl.holes_begin(fit), hend = expl.holes_end(fit);
	for (int i = 0; hit != hend; hit++, i++, f++) {
		ds.make_set(f);
		if (DEBUG)
			std::cerr << "Hole " << i << " (" << hit->mark() << ") \n";
		// if (!hit->mark() && !DEBUG)
		//         continue;

		Hfc_circulator hhafc(hit), hdone(hit);
		// NefPoly trace;
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
					// trace += line;
					// INIT_BITS(f, poly2fis[line]);
					// FOR_BITS(i, poly2fis[line])
					// {
					//         ds.union_set(f, i);
					// }
					// poly2fis[line][f] = 1;
					BOOST_FOREACH (int i, poly2fis[line]) {
						ds.union_set(f, i);
					}
					poly2fis[line].push_back(f);
					if (fi2poly.find(f) == fi2poly.end())
						fi2poly[f] = line;
					else
						fi2poly[f] += line;
				}
				if (vert < poly) {
					// trace += vert;
					// INIT_BITS(f, poly2fis[vert]);
					// FOR_BITS(i, poly2fis[vert])
					// {
					//         ds.union_set(f, i);
					// }
					// poly2fis[vert][f] = 1;
					BOOST_FOREACH (int i, poly2fis[vert]) {
						ds.union_set(f, i);
					}
					poly2fis[vert].push_back(f);
					if (fi2poly.find(f) == fi2poly.end())
						fi2poly[f] = vert;
					else
						fi2poly[f] += vert;
				} else {
					f++;
					ds.make_set(f);
				}
			}
			hhafc--;
		} while (hhafc != hdone);
		NefPoly outer(++opoints.begin(), opoints.end(), EXCLUDED);
		poly2fis[outer].size();
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
	        .def("to_list", &Region::to_list);
	// py::def(
	//	   "blah", +[]() { return "blah"; });
}
