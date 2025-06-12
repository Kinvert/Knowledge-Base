# 🌍 KMZ Files

**KMZ files** are a common format used to store geographic data, primarily for use in applications like **Google Earth**. They are essentially compressed versions of **[[KML]] (Keyhole Markup Language)** files, often bundling additional resources such as images, icons, and 3D models into a single archive.

---

## 📝 Summary

- **Full Name**: Keyhole Markup Zipped
- **Based On**: KML (Keyhole Markup Language) — XML-based format
- **Common Uses**: 
  - Visualizing GPS tracks
  - Mapping waypoints, routes, and paths
  - 3D models of buildings or terrain
  - Embedding custom imagery or icons into maps

---

## 🔄 File Structure

KMZ is essentially a ZIP file with a `.kmz` extension. Inside it typically contains:

| File | Purpose |
|------|---------|
| `doc.kml` | The main KML file containing geographic data |
| `/files/` | Folder containing images, icons, or other resources |
| `models/` | Optional folder containing 3D models (e.g., COLLADA `.dae` files) |

---

## 🌐 Common Applications That Open KMZ

| Application | Platform | Notes |
|--------------|----------|-------|
| [[Google Earth]] | Windows, Mac, Linux, Web | Primary application |
| Google Maps | Web | Limited KMZ support |
| [[QGIS]] | Windows, Mac, Linux | Advanced GIS software |
| [[ArcGIS]] | Windows | GIS professional tool |
| Garmin BaseCamp | Windows, Mac | Import/export for GPS units |
| Mobile Apps | Android/iOS | Various 3rd party apps support KMZ |

---

## 🛠 How It Works

- Uses **WGS84** coordinate system (latitude, longitude, altitude).
- Can embed **placemarks, polygons, paths, overlays, and 3D objects**.
- Supports **time-based data** (animation of vehicle movement, historical data).
- Supports **altitude modes**:
  - *absolute* (fixed altitude)
  - *relativeToGround* (height above terrain)
  - *clampToGround* (stick to ground surface)

---

## 📦 KMZ vs KML

| Feature | KML | KMZ |
|---------|-----|-----|
| File Type | XML (plaintext) | ZIP (compressed) |
| Can include images/models | ❌ | ✅ |
| Human-readable | ✅ | ❌ |
| File size | Larger | Smaller |
| Sharing | Less convenient | More convenient |

---

## 📊 Comparable Formats

| Format | Similarity | Use Case |
|--------|------------|----------|
| GPX | Waypoints & tracks | Hiking, GPS devices |
| SHP (Shapefile) | GIS data | Professional mapping |
| GeoJSON | Lightweight web format | Web mapping apps |
| GML | XML-based like KML | Advanced GIS |
| CSV | Tabular data with lat/long | Simple coordinates |

---

## ✅ Strengths

- Compressed: smaller file sizes.
- Bundles resources into one file.
- Widely supported by consumer mapping software.
- Easy to share via email or download.

---

## ❌ Limitations

- Primarily intended for visualization, not complex analysis.
- Limited support for coordinate systems (WGS84 only).
- Performance issues with very large datasets.
- Limited versioning and schema evolution.

---

## 🧑‍💻 How to Create KMZ Files

- **Google Earth Pro** → Save Place As → Choose KMZ.
- **GIS Tools** (QGIS, ArcGIS) → Export as KMZ.
- **Manual**: Create KML file + assets → Zip → Rename extension to `.kmz`.

---

## ⚙️ Typical Use Cases

- Drone flight paths
- Surveying data visualization
- Historical imagery overlays
- Custom Google Earth tours
- Asset tracking (fleet, aircraft, ships)
- Amateur radio APRS mapping
- Hiking and biking trail sharing

---

## 🚩 Related Standards and Specs

- **OGC KML Standard**: Open Geospatial Consortium (OGC) adopted KML as a standard.
- **WGS84**: World Geodetic System 1984 coordinate system.

---

## 🔗 References

- [Google Developers - KML Documentation](https://developers.google.com/kml/documentation/)
- [OGC KML Standard](https://www.ogc.org/standards/kml)
- [QGIS KMZ Support](https://qgis.org/)
- [Google Earth Pro Download](https://www.google.com/earth/versions/#earth-pro)
