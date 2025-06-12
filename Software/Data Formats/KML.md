# 🌎 KML (Keyhole Markup Language)

**KML (Keyhole Markup Language)** is an XML-based format used to describe geographic data and visualization features. It was originally developed by **Keyhole Inc.** (acquired by Google in 2004) and is the native format for **Google Earth** and many other geospatial applications.

---

## 📝 Summary

- **Full Name**: Keyhole Markup Language
- **File Extension**: `.kml`
- **Standardized by**: [[Open Geospatial Consortium (OGC)]]
- **Main Purpose**: Describing geographical features, annotations, and visualization instructions for virtual globes and maps.
- **Related Format**: [[KMZ]] (compressed version of KML)

---

## 🌐 Common Applications That Support KML

| Application | Platform | Notes |
|--------------|----------|-------|
| [[Google Earth]] | Windows, Mac, Linux, Web | Primary viewer |
| Google Maps | Web | Can import KML layers |
| [[QGIS]] | Windows, Mac, Linux | Open-source GIS software |
| [[ArcGIS]] | Windows | Professional GIS tool |
| GPSBabel | Windows, Mac, Linux | Format conversion |
| Mobile Apps | Android/iOS | Many apps support KML |

---

## 📦 File Structure

A typical KML file is structured as plain text XML and may contain:

- **Placemark**: A point, line, or polygon location
- **Path**: A series of points forming a route
- **Polygon**: Closed shape area
- **Overlay**: Image overlaid on the globe
- **NetworkLink**: Links to other KML files online
- **Folder**: Logical grouping of KML elements
- **Style**: Definitions for colors, icons, line styles, etc.
- **TimeSpan / TimeStamp**: Time-based animations

---

## 📊 Comparable Formats

| Format | Similarity | Use Case |
|--------|------------|----------|
| [[KMZ]] | Directly related | Compressed KML with resources |
| [[GPX]] | Waypoints & tracks | Fitness, hiking, GPS devices |
| [[GeoJSON]] | Web-based | Web mapping & APIs |
| [[Shapefile]] | GIS professional | Detailed GIS data layers |
| [[GML]] | XML-based GIS | Professional GIS systems |

---

## 📌 Strengths

- Human-readable and editable (XML-based).
- Open standard (OGC).
- Widely supported across many platforms.
- Easy to share via email, websites, or cloud storage.
- Can embed visualization styles, icons, and time-based data.

---

## ⚠️ Limitations

- Limited to WGS84 coordinate system.
- [[XML]] format may be bulky for very large datasets.
- Performance may degrade with massive files.
- Not as feature-rich for complex GIS analysis.

---

## 🏷 Supported Features

| Feature | Supported? |
|---------|-------------|
| Points/Lines/Polygons | ✅ |
| Image Overlays | ✅ |
| 3D Models | ✅ |
| Network Links | ✅ |
| Timestamps & Animations | ✅ |
| Styles & Icons | ✅ |
| Complex Topologies | ❌ (Use Shapefile, GML instead) |
| Advanced GIS Analysis | ❌ |

---

## ⚙️ How to Create/Edit

- **Google Earth Pro** → Add placemarks → Save as KML.
- **QGIS / ArcGIS** → Export functions.
- **Text Editors** → Manually create or edit XML.
- **Web Tools** → Many free online KML editors exist.

---

## 🛰 Common Use Cases

- Drone flight path planning and visualization.
- Surveying and fieldwork data.
- Historical time-based data visualization.
- Real estate mapping and property boundary definition.
- Amateur radio mapping and APRS visualizations.
- Hiking and cycling route sharing.
- Asset tracking (shipping, aviation, vehicle fleets).

---

## 🌐 Standards & Specs

- **Coordinate System**: WGS84 (EPSG:4326)
- **Standardization Body**: [[Open Geospatial Consortium (OGC)]]
- **Schema Version**: KML 2.2 is most widely used.

---

## 🔗 Internal Links

- See also: [[KMZ]], [[GeoJSON]], [[GPX]], [[Shapefile]], [[Geospatial Data Formats]]
- Related hardware: [[GPS Units]], [[Drones]], [[Survey Equipment]]

---

## 📚 References

- [Google Developers - KML Documentation](https://developers.google.com/kml/documentation)
- [OGC KML Standard](https://www.ogc.org/standards/kml)
- [QGIS Official Site](https://qgis.org/)
