// BEGIN HACK
#include <CGAL/IO/io_impl.h>
#include <CGAL/assertions_impl.h>
// END HACK
#include <CGAL/circulator.h>
#include <CGAL/Exact_integer.h>
#include <CGAL/Nef_polyhedron_2.h>
#include <CGAL/number_utils_classes.h>
#include <CGAL/Filtered_extended_homogeneous.h>
#include <boost/python.hpp>
#include <boost/python/list.hpp>
#include <boost/python/iterator.hpp>
#include <boost/container_hash/hash.hpp>
#include <boost/unordered/unordered_map_fwd.hpp>
#include <boost/unordered_map.hpp>
#include <vector>
#include <cstddef>
#include <iostream>
#include <iterator>
#include <unordered_set>

#define DEBUG (true)

namespace py = boost::python;

typedef CGAL::Exact_integer RT;
typedef CGAL::Filtered_extended_homogeneous<RT> Extended_kernel;
typedef CGAL::Nef_polyhedron_2<Extended_kernel> Nef_polyhedron;
typedef Nef_polyhedron::Point Point;
typedef Nef_polyhedron::Line Line;
typedef Nef_polyhedron::Explorer Explorer;
typedef Explorer::Vertex_const_handle Vert;
typedef Explorer::Halfedge_around_face_const_circulator Hfc_circulator;
typedef Explorer::Isolated_vertex_const_iterator Iso_Vert_it;
typedef Explorer::Face_const_iterator Face_it;
typedef Explorer::Hole_const_iterator Hole_it;
// typedef Explorer::Halfedge_const_iterator Hedge_it;
// typedef Explorer::Vertex_const_iterator Vert_it;
typedef CGAL::Circulator_from_iterator<std::vector<Point>::iterator> Cyclic_Points;

#define EXCLUDED (Nef_polyhedron::Boundary::EXCLUDED)

struct Region {
	Nef_polyhedron poly;
	static const Nef_polyhedron null;

	Region()
	{
		poly = Nef_polyhedron(Nef_polyhedron::EMPTY);
	}

	Region(const py::list &pypoly)
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
		poly = Nef_polyhedron(points, points + len(pypoly));
	}

	Region(const Nef_polyhedron &init_poly)
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
		// return poly.explorer().number_of_connected_components();
		if (poly.is_empty())
			return 0;
		Explorer expl = poly.explorer();
		Face_it fit = ++expl.faces_begin();
		Hole_it hit = expl.holes_begin(fit), end = expl.holes_end(fit);
		return std::distance(hit, end);
	}

	py::list get_components();

	bool operator==(Region const &b) const
	{
		return poly == b.poly;
	}

	friend std::size_t hash_value(Region const &r)
	{
		std::size_t seed = 0;
		Explorer expl = r.poly.explorer();
		Explorer::Point_const_iterator pit = expl.points_begin(), pend = expl.points_end();
		for (; pit != pend; pit++) {
			if (pit->is_standard()) {
				int x = (int)expl.point(pit).hx();
				int y = (int)expl.point(pit).hy();
				boost::hash_combine(seed, x);
				boost::hash_combine(seed, y);
			}
		}

		return seed;
	}
};
const Nef_polyhedron Region::null = Nef_polyhedron(Nef_polyhedron::EMPTY);
py::list Region::get_components()
{
	py::list pypoly = py::list();
	if (poly.is_empty())
		return pypoly;

	// std::unordered_set<Region, boost::hash<Region> > con_comps;
	boost::unordered_map<Region, int, boost::hash<Region> > components;
	Explorer expl = poly.explorer();
	Face_it fit = ++expl.faces_begin(), fend = expl.faces_end();

	if (DEBUG)
		std::cerr << "--- Isolated Vertices ---\n";
	Iso_Vert_it ivit = expl.isolated_vertices_begin(fit), ivend = expl.isolated_vertices_end(fit);
	for (int i = 0; ivit != ivend; ivit++, i++) {
		if (DEBUG)
			std::cerr << "Vert " << i << ": " << expl.point(ivit) << ", " << ivit->mark() << '\n';
		Point point[1] = {expl.point(ivit)};
		// con_comps.insert(Region(Nef_polyhedron(point, point + 1)));
		components[Region(Nef_polyhedron(point, point + 1))] = 1;
	}

	Nef_polyhedron all_faces;

	fit++;
	if (DEBUG)
		std::cerr << "--- Faces ---\n";
	for (int i = 0; fit != fend; fit++, i++) {
		if (DEBUG)
			std::cerr << "Face " << i << " (" << fit->mark() << "):\n";
		if (!fit->mark() && !DEBUG)
			continue;

		Hfc_circulator hafc = expl.face_cycle(fit);
		if (hafc != Hfc_circulator()) {
			Hfc_circulator done(hafc);
			Nef_polyhedron boundary;
			std::vector<Point> points;
			Vert src = expl.source(hafc);
			if (expl.is_standard(src)) {
				if (DEBUG)
					std::cerr << '\t' << expl.point(src) << ", " << src->mark() << '\n';
				points.push_back(expl.point(src));
			}
			do {
				Vert tgt = expl.target(hafc);
				if (expl.is_standard(tgt)) {
					if (DEBUG)
						std::cerr << '\t' << expl.point(tgt) << ", " << tgt->mark() << '\n';
					points.push_back(expl.point(tgt));
					Nef_polyhedron line = Nef_polyhedron(----points.end(), points.end()) -
					                      Nef_polyhedron(----points.end(), --points.end()) -
					                      Nef_polyhedron(--points.end(), points.end());
					if (DEBUG)
						std::cerr << "\t\t L__ e: " << (line < poly) << '\n';
					if (line < poly)
						boundary += line;
					if (tgt->mark())
						boundary += Nef_polyhedron(--points.end(), points.end());
				}
				hafc++;
			} while (hafc != done);
			Region face(Nef_polyhedron(++points.begin(), points.end(), EXCLUDED) + boundary);

			if (DEBUG)
				std::cerr << components[face] << '\n';
			if (components[face])
				continue;

			Nef_polyhedron to_remove;

			Iso_Vert_it ivit = expl.isolated_vertices_begin(fit), ivend = expl.isolated_vertices_end(fit);
			for (int i = 0; ivit != ivend; ivit++, i++) {
				if (DEBUG)
					std::cerr << "\tVert " << i << ": " << expl.point(ivit) << ", " << ivit->mark()
					          << '\n';
				Point point[1] = {expl.point(ivit)};
				to_remove += Nef_polyhedron(point, point + 1);
			}

			Hole_it hit = expl.holes_begin(fit), hend = expl.holes_end(fit);
			for (int i = 0; hit != hend; hit++, i++) {
				if (DEBUG)
					std::cerr << "\tHole " << i << " (" << hit->mark() << "):\n";
				Hfc_circulator hafc(hit), done(hit);
				Nef_polyhedron boundary;
				std::vector<Point> points;
				Vert tgt = expl.target(hafc);
				if (expl.is_standard(tgt)) {
					if (DEBUG)
						std::cerr << "\t\t" << expl.point(tgt) << ", " << tgt->mark() << '\n';
					points.push_back(expl.point(tgt));
				}
				do {
					Vert src = expl.source(hafc);
					if (expl.is_standard(src)) {
						if (DEBUG)
							std::cerr << "\t\t" << expl.point(src) << ", " << src->mark()
							          << '\n';
						points.push_back(expl.point(src));
						Nef_polyhedron line = Nef_polyhedron(----points.end(), points.end()) -
						                      Nef_polyhedron(----points.end(), --points.end()) -
						                      Nef_polyhedron(--points.end(), points.end());
						if (DEBUG)
							std::cerr << "\t\t L__ e: " << (line < poly) << '\n';
						if (line < poly)
							boundary += line;
						if (src->mark())
							boundary += Nef_polyhedron(--points.end(), points.end());
					}
					hafc--;
				} while (hafc != done);
				Region hole(Nef_polyhedron(++points.begin(), points.end(), EXCLUDED) + boundary);
				assert(components[hole] == 0);
				components[hole] = -1;
				to_remove += Nef_polyhedron(++points.begin(), points.end()) - boundary;
			}

			all_faces += face.poly;
			if (!to_remove.is_empty()) {
				components[face] = -1;
				face = Region(face.poly - to_remove);
			}
			components[face] = 1;
		}
	}

	if (DEBUG)
		std::cerr << "--- Isolated Edges ---\n";

	fit = ++expl.faces_begin();
	Hole_it hit = expl.holes_begin(fit), hend = expl.holes_end(fit);
	for (int i = 0; hit != hend; hit++, i++) {
		if (DEBUG)
			std::cerr << "--- Hole " << i << " (" << hit->mark() << ", "
			          << ") ---\n";
		Hfc_circulator hafc(hit), done(hit);
		Nef_polyhedron trace;
		std::vector<Point> points;
		Vert tgt = expl.target(hafc);
		if (expl.is_standard(tgt)) {
			if (DEBUG)
				std::cerr << '\t' << expl.point(tgt) << ", " << tgt->mark() << '\n';
			points.push_back(expl.point(tgt));
		}
		do {
			Vert src = expl.source(hafc);
			if (expl.is_standard(src)) {
				if (DEBUG)
					std::cerr << "\t" << expl.point(src) << ", " << src->mark() << '\n';
				points.push_back(expl.point(src));
				Nef_polyhedron line = Nef_polyhedron(----points.end(), points.end()) -
				                      Nef_polyhedron(----points.end(), --points.end()) -
				                      Nef_polyhedron(--points.end(), points.end());
				if (DEBUG)
					std::cerr << "\t\t L__ e: " << (line < poly) << '\n';
				if (line < poly)
					trace += line;
				if (src->mark())
					trace += Nef_polyhedron(--points.end(), points.end());
			}
			hafc--;
		} while (hafc != done);
		Region comp_candidate(Nef_polyhedron(++points.begin(), points.end(), EXCLUDED) + trace - all_faces);
		if (!comp_candidate.is_empty())
			components[comp_candidate] = 1;
	}

	for (auto rit = components.begin(); rit != components.end(); rit++)
		if (rit->second > 0)
			pypoly.append(rit->first);
	return pypoly;
}

BOOST_PYTHON_MODULE(region)
{
	py::class_<Region>("region", py::init<>())
	        .def(py::init<py::list &>())
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
	        .def("get_components", &Region::get_components);
	// py::def(
	//	   "blah", +[]() { return "blah"; });
}
