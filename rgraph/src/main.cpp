#include <iostream>
#include <valgrind/callgrind.h>
#include "region.cpp"

int Region::test()
{
	// const bool DEBUG = true;
	// py::list pypoly = py::list();
	if (poly.is_empty())
		return 0;

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
		if (!fit->mark() && !DEBUG)
			continue;

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
		// pypoly.append(Region(NefPoly(point, point + 1)));
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
		// if (fi == pi)
		//         pypoly.append(Region(fip->second));
	}

	return 8;
}

int main()
{
	std::cerr << "Start\n";
	Point x1(50, 50);
	Point x2(950, 50);
	Point x3(950, 950);
	Point x4(50, 950);
	Point y1(784, 578);
	Point y2(798, 528);
	Point y3(834, 492);
	Point y4(884, 478);
	Point y5(934, 492);
	Point y6(970, 528);
	Point y7(984, 578);
	Point y8(970, 626);
	Point y9(934, 664);
	Point y10(884, 678);
	Point y11(836, 664);
	Point y12(798, 626);
	std::cerr << "Points\n";
	Point ps1[4] = {x1, x2, x3, x4};
	Point ps2[12] = {y1, y2, y3, y4, y5, y6, y7, y8, y9, y10, y11, y12};
	std::cerr << "Shapes\n";
	NefPoly p1(ps1, ps1 + 4);
	NefPoly p2(ps2, ps2 + 12);
	std::cerr << "Polys\n";
	NefPoly p3 = p1.intersection(p2);
	std::cerr << "Intersection\n";

	Region r(p3);
	CALLGRIND_START_INSTRUMENTATION;
	CALLGRIND_TOGGLE_COLLECT;
	int test = r.test();
	CALLGRIND_TOGGLE_COLLECT;
	CALLGRIND_STOP_INSTRUMENTATION;
	std::cerr << test << '\n';

	return 0;
}
