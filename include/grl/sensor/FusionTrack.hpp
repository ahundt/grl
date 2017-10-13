#ifndef GRL_ATRACSYS_FUSION_TRACK
#define GRL_ATRACSYS_FUSION_TRACK

#include <tuple>
#include <string>
#include <vector>
#include <system_error>
#include <algorithm>
#include <memory>

// FusionTrack Libraries
#include <ftkInterface.h>
#include <geometryHelper.hpp> // I would like to get rid of this and use only ftk functions + remove atracsys_DIR/bin from include directories

#include "grl/TimeEvent.hpp"

#ifdef HAVE_SPDLOG
/// The spdlog library https://github.com/gabime/spdlog
#include <spdlog/spdlog.h>
#endif // HAVE_SPDLOG

#ifndef BOOST_THROW_EXCEPTION
#define BOOST_THROW_EXCEPTION(x) throw x
#endif

namespace grl { namespace sensor {

namespace detail {
    void updateDeviceSerialNumber(uint64 device, void * user, ftkDeviceType type);
}

/// @class FusionTrack provides a single threaded driver for
/// one or more Atracsys FusionTrack optical tracker devices, which
/// provides fast updates to the pose of objects in front of it such
/// as a fiducial or a marker.
///
/// A fiducial is typically a reflective sphere or infrared LED.
/// A marker is typically metal frame with multiple spheres treated as a single unit
///
/// This driver is designed to be extremely easy to use, first create a Params struct,
/// which can be default constructed or made with FusionTrack::defaultParams.
///
/// Next specify the filenames of the ini files which store the marker geometry
/// including the 3d origin and position of each reflector or LED on the marker,
/// or specify them in code. Then construct a FusionTrack object and a FusionTrack::Frame,
/// finally, simply call FusionTrack::receive(receive) every time you want updated data.
/// It is important to note that if you call the update on an extremely fast loop,
/// the ReceivedData object may simply have an error code set indicating no new data was
/// available.
class FusionTrack {
public:

    /// Note that some params will be modified during the initialization of the FusionTrack object.
    /// In particular, any geometryFilenames will be loaded and added to the list of markerModelGeometries.
    struct Params {
        /// Maximum time the API can block waiting on data before giving up
        uint64_t blockLimitMilliseconds = 100;
        /// Maximum number of attempts to connect to any FusionTrack device before giving up
        uint64_t maximumConnectionAttempts = 10;
        /// Geometries aka fiducials aka markers to be loaded from ini files.
        /// The data loaded should not repeat IDs from MarkerIDs.
        std::vector<std::string> geometryFilenames;
        /// Path to the directory with the marker ini files listed above
        /// Uses the default current working directory if empty
        std::string geometryDir;
        /// Optional list of optical tracker device ids to expect
        /// will be loaded automatically if empty
        std::vector<uint64_t> TrackerDeviceIDs;
        /// Marker geometry unique integer IDs.
        /// This ids the shape of a marker and more than marker can have
        /// an identical shape.
        std::vector<uint64_t> markerIDs;
        /// Optional Marker geometry names with one for each ID, none otherwise
        std::vector<std::string> markerNames;

        /// A MarkerModelGeometry is a std::vector<ftk3DPoint>
        /// which contains the dimensions from the origin point of a marker
        /// (metal frame with multiple spheres or infrared LEDs)
        /// to the center of each fiducial attached to the marker.
        /// Often there is a physical indicator on the marker such as an
        /// arrow or metal pointer indicating the location of the origin.
        /// ftk3DPoint simply has 3 floats x,y,z. Note that if two identical
        /// physical marker devices are visible, two instances of the same geometry
        /// will be visible at once.
        ///
        /// @see FusionTrackToEigen.hpp for a converter to and from the
        ///      widely used Eigen::Vector3f class.
        typedef std::vector<ftk3DPoint> MarkerModelGeometry;

        /// There should be one marker model geometry for each marker
        /// with physically different dimensions.
        /// Distances here are measured in mm.
        std::vector<MarkerModelGeometry> markerModelGeometries;
        /// Should the pixels from the left camera on each device be supplied to the user?
        /// (not yet supported)
        bool retrieveLeftPixels = false;
        /// Should the pixels from the right camera on each device be supplied to the user?
        /// (not yet supported)
        bool retrieveRightPixels = false;

        /// maxLeftImageRegionOfInterestBoxes is the total maximum number of
        /// image pixel boxes (aka ftkRawData or region of interest)
        /// to allocate in a Frame for the left camera.
        /// Recommended values are 0 if you don't use this or 128 if you do.
        uint32_t maxLeftImageRegionOfInterestBoxes = 0;

        /// maxRightImageRegionOfInterestBoxes is the total maximum number of
        /// image pixel boxes (aka ftkRawData or region of interest)
        /// to allocate in a Frame for the right camera.
        /// Recommended values are 0 if you don't use this or 128 if you do.
        uint32_t maxRightImageRegionOfInterestBoxes = 0;
        uint32_t max3DFiducialInstances = 128;
        uint32_t maxMarkerInstances = 32;

        /// Name for this connection / FusionTrack driver instance
        /// useful for debugging and when multiple data sources are used
        std::string name;

        /// Name for the clock on the FusionTrack
        /// Useful for timing calculations and debugging.
        std::string deviceClockID;

        /// Name for the local clock on which this driver runs
        /// Useful for timing calculations and debugging.
        std::string localClockID;

        #ifdef HAVE_SPDLOG
        /// The spdlog logger library https://github.com/gabime/spdlog
        /// is optional but recommended for fast data and error
        /// logging. Enabling spdlog provides useful error messages
        /// and minimizes the effect on runtime performance
        /// of logging, since std::cout can increase processing times
        /// substantially. The empty string will turn logging off,
        /// otherwise this string should be the name of the logger,
        /// and "console" for the default logger.
        std::string loggerName;
        #endif // HAVE_SPDLOG
    };

    static const Params emptyDefaultParams(){
        Params params;
        params.blockLimitMilliseconds = 100;
        params.maximumConnectionAttempts = 10;
        params.name = "/FusionTrack";
        params.localClockID = "/control_computer/clock/steady";
        params.deviceClockID = "/FusionTrack";
        return params;
    }

    static const Params defaultParams(){
        Params params = emptyDefaultParams();

        std::vector<std::string> geometries;
        geometries.push_back("geometry0022.ini");
        geometries.push_back("geometry0055.ini");
        params.geometryFilenames = geometries;
        return params;
    }

    /// A custom clock for the FusionTrack microsecond time stamps
    struct MicrosecondClock {
      using rep = int64_t;
      /// 1 microsecond
      using period = std::ratio<1, 1000000>;
      using duration = std::chrono::duration<rep, period>;
      using time_point = std::chrono::time_point<MicrosecondClock>;
      static constexpr bool is_steady = true;

        /// TODO(ahundt) currently assuming the FusionTrack timestamp is from the unix time epoch
        static time_point now() noexcept
        {
            using namespace std::chrono;
            return time_point
              (
                duration_cast<duration>(system_clock::now().time_since_epoch())
              );
        }
    };

    /// TODO(ahundt) currently assuming the FusionTrack timestamp is from the unix time epoch
    cartographer::common::Time FusionTrackTimeToCommonTime(typename MicrosecondClock::time_point FTtime) {
        return cartographer::common::Time(
            std::chrono::duration_cast<cartographer::common::UniversalTimeScaleClock::duration>(FTtime.time_since_epoch()) +
                std::chrono::seconds(cartographer::common::kUtsEpochOffsetFromUnixEpochInSeconds)
        );
    }


    cartographer::common::Time ImageHeaderToCommonTime(const ::ftkImageHeader& tq) {
        typename MicrosecondClock::time_point fttp(MicrosecondClock::duration(tq.timestampUS));
        return FusionTrackTimeToCommonTime(fttp);
    }

    /// typedef std::vector<ftk3DPoint> MarkerModelGeometry;
    /// std::vector<MarkerModelGeometry> markerModelGeometries;
    Params::MarkerModelGeometry ftkGeometryToMarkerModelGeometry(::ftkGeometry& ftkg){
        Params::MarkerModelGeometry geom;
        for(int i = 0; i < ftkg.pointsCount; ++i)
        {
            geom.push_back(ftkg.positions[i]);
        }
        return geom;
    }


    FusionTrack(Params params = defaultParams()):
      m_params(params)
    {
        #ifdef HAVE_SPDLOG
        if(!params.loggerName.empty()){
            m_logger = spdlog::get(params.loggerName);
        }

        #endif // HAVE_SPDLOG
        ftkError error;
        // search for devices
        for(std::size_t i = 0; i < m_params.maximumConnectionAttempts; i++) {
          m_ftkLibrary = ftkInit();
          // try a number of times before giving up
          error = ftkEnumerateDevices(m_ftkLibrary,
                                      detail::updateDeviceSerialNumber,
                                      this);
          if (error == FTK_OK && m_deviceSerialNumbers.size() != 0) {
            break;
          } else {
            ftkClose(&m_ftkLibrary);
            m_ftkLibrary = nullptr;
          }

        }

        if (error != FTK_OK) {
            if(m_ftkLibrary != nullptr) ftkClose(&m_ftkLibrary);
            BOOST_THROW_EXCEPTION(std::system_error(
                std::make_error_code(std::errc::no_such_device),
                std::string("FusionTrack: unable to enumerate devices (FusionTrack)" )));
        }
        if (m_deviceSerialNumbers.size() == 0 || m_deviceSerialNumbers[0] == 0) {
            if(m_ftkLibrary != nullptr) ftkClose(&m_ftkLibrary);
            BOOST_THROW_EXCEPTION(std::system_error(
                std::make_error_code(std::errc::no_such_device),
                std::string("FusionTrack: no device connected (FusionTrack)" )));
        }

        /// TODO(ahundt) call loadGeometry for geometries already defined in params object using: TrackerDeviceIDs markerIDs markerNames markerModelGeometries

        // make sure we can find and load this tool ini file
        // data loaded from the ini file is also placed back in params
        for(auto fileName : m_params.geometryFilenames){
            ftkGeometry geometry;
            switch (loadGeometry(m_ftkLibrary, m_deviceSerialNumbers[0], fileName, geometry))
            {
            case 1:
                BOOST_THROW_EXCEPTION(std::runtime_error(std::string("FusionTrack: loaded ") + fileName + " from installation directory"));
            case 0:
                for( auto serialNumber : m_deviceSerialNumbers)
                {
                    error = ftkSetGeometry(m_ftkLibrary, serialNumber, &geometry);
                    if (error != FTK_OK) {
                        BOOST_THROW_EXCEPTION(std::runtime_error(std::string("FusionTrack: unable to set geometry for tool ") + fileName + " (FusionTrack)" ));
                    }
                }
                m_params.markerIDs.push_back(geometry.geometryId);
                m_params.markerNames.push_back(fileName);
                m_params.markerModelGeometries.push_back(ftkGeometryToMarkerModelGeometry(geometry));
                break;
            default:
                BOOST_THROW_EXCEPTION(std::runtime_error(std::string( "FusionTrack: error, cannot load geometry file ") + fileName ));
            }
        }
    }

    /// Frame stores FusionTrack data captured at a single point in time,
    /// it includes time stamps, the serial number of the device that captured
    /// the data, the actual image data,
    /// each fiducial (reflective sphere or infrared LED)
    /// each marker (metal frame with multiple spheres),
    /// probabilities of an accurate detection, errors, etc.
    /// It is important to note that if you call the update on an extremely fast loop,
    /// the FusionTrack::Frame object may simply have an error code set
    /// indicating no new data was available.
    ///
    /// Creating a Frame involves memory allocation, so it is recommended that you
    /// initialize a fixed number of frames you will use at startup
    /// to minimize the performance effects caused by allocation.
    class Frame {
      public:
        static const uint8_t Version = 0;
        static const std::size_t CameraImageSize = 2048u*1088u;
        typedef std::array<uint8_t, CameraImageSize> CameraImage;

        /// Initialize Frame to contain a frame in a way equivalent
        /// to what is done in ftkCreateFrame and ftkSetFrameOptions.
        ///
        /// @param serialNumber the serial number of the FusionTrack device you wish to receive data from,
        ///        the default of 0 will use the default device. You should only need to set this if more
        ///        than one FusionTrack device will be connected.
        /// @param maxLeftImageRegionOfInterestBoxes is the total maximum number of
        ///        image pixel boxes (aka ftkRawData or region of interest)
        ///        to allocate in a Frame for the left camera.
        ///        Recommended values are 0 if you don't use this or 128 if you do.
        /// @param maxLeftImageRegionOfInterestBoxes is the total maximum number of
        ///        image pixel boxes (aka ftkRawData or region of interest)
        ///        to allocate in a Frame for the left camera.
        ///        Recommended values are 0 if you don't use this or 128 if you do.
        /// @param maxRightImageRegionOfInterestBoxes location and number of pixels which should contain a fiducial. Recommended values are 0 or 128.
        Frame(
            uint64_t serialNumber = 0,
            bool retrieveLeftPixels = false,
            bool retrieveRightPixels = false,
            uint32_t maxLeftImageRegionOfInterestBoxes = 0,
            uint32_t maxRightImageRegionOfInterestBoxes = 0,
            uint32_t max3DFiducialInstances = 128,
            uint32_t maxMarkerInstances = 32):
            SerialNumber(serialNumber),
            FrameQueryP(ftkCreateFrame()),
            Error(FTK_OK)
        {

            if(FrameQueryP == nullptr) throw std::bad_alloc();

            // image header
            FrameQueryP->imageHeader = &imageHeader;
            FrameQueryP->imageHeaderVersionSize.Version = Version;
            FrameQueryP->imageHeaderVersionSize.ReservedSize = sizeof(ftkImageHeader);

            // actual left camera image
            if(retrieveLeftPixels){
                CameraImageLeftP = std::make_shared<CameraImage>();
                FrameQueryP->imageLeftPixels = CameraImageLeftP->begin();
                FrameQueryP->imageLeftVersionSize.Version = Version;
                FrameQueryP->imageLeftVersionSize.ReservedSize = sizeof(ftkMarker) * Markers.size();
            } else {
                FrameQueryP->imageLeftPixels = nullptr;
                FrameQueryP->imageLeftVersionSize.Version = Version;
                FrameQueryP->imageLeftVersionSize.ReservedSize = 0;
            }

            // actual right camera image
            if(retrieveRightPixels){
                CameraImageRightP = std::make_shared<CameraImage>();
                FrameQueryP->imageRightPixels = CameraImageRightP->begin();
                FrameQueryP->imageRightVersionSize.Version = Version;
                FrameQueryP->imageRightVersionSize.ReservedSize = sizeof(ftkMarker) * Markers.size();
            } else {
                FrameQueryP->imageRightPixels = nullptr;
                FrameQueryP->imageRightVersionSize.Version = Version;
                FrameQueryP->imageRightVersionSize.ReservedSize = 0;
            }

            // boxes around potential fiducials in left image (ftkRawData)
            ImageRegionOfInterestBoxesLeft.reserve(maxLeftImageRegionOfInterestBoxes);
            FrameQueryP->rawDataLeft = &ImageRegionOfInterestBoxesLeft[0];
            FrameQueryP->rawDataLeftVersionSize.Version = Version;
            FrameQueryP->rawDataLeftVersionSize.ReservedSize = sizeof(ftkRawData) * maxLeftImageRegionOfInterestBoxes;

            // boxes around potential fiducials in left image (ftkRawData)
            ImageRegionOfInterestBoxesRight.reserve(maxRightImageRegionOfInterestBoxes);
            FrameQueryP->rawDataRight = &ImageRegionOfInterestBoxesRight[0];
            FrameQueryP->rawDataRightVersionSize.Version = Version;
            FrameQueryP->rawDataRightVersionSize.ReservedSize = sizeof(ftkRawData) * maxRightImageRegionOfInterestBoxes;

            Fiducials.reserve(max3DFiducialInstances);
            FrameQueryP->threeDFiducials = &Fiducials[0];
            FrameQueryP->threeDFiducialsVersionSize.Version = Version;
            FrameQueryP->threeDFiducialsVersionSize.ReservedSize = sizeof(ftk3DFiducial) * max3DFiducialInstances;

            Markers.reserve(maxMarkerInstances);
            FrameQueryP->markers = &Markers[0];
            FrameQueryP->markersVersionSize.Version = Version;
            FrameQueryP->markersVersionSize.ReservedSize = sizeof(ftkMarker) * maxMarkerInstances;
        }

        /// The serial number of the device that produced this data
        uint64_t SerialNumber;

        /// Device Type identifier, the physical model being used,
        /// such as a FusionTrack 500, 250, simulator, etc.
        ftkDeviceType DeviceType;

        /// Contains all the data pointers and status information
        /// used internally within the FusionTrack library.
        /// @see ftkFrameQuery
        std::shared_ptr<ftkFrameQuery> FrameQueryP;

        /// Byte array used to store a camera image
        std::shared_ptr<CameraImage> CameraImageLeftP;

        /// Byte array used to store a camera image
        std::shared_ptr<CameraImage> CameraImageRightP;

        /// Contains the image dimensions, image counter,
        /// and the internal timestamp information.
        ftkImageHeader imageHeader;

        /// Contains position and other data data for each detected fiducial,
        /// A fiducial is a reflective sphere or LED light.
        /// @see ftk3DFiducial
        std::vector<ftk3DFiducial> Fiducials;

        /// Contains data and pose (3D position and rotation) information for each detected marker.
        /// A marker is typically metal frame with multiple spheres treated as a single unit,
        /// used to track position and orientation.
        std::vector<ftkMarker> Markers;

        /// Image pixel boxes (aka ftkRawData or region of interest) is the x, y location
        /// in the camera image of a suspected fiducial, plus the status
        /// and total count of pixels in the box, and the probability it
        /// is a fiducial.
        std::vector<ftkRawData> ImageRegionOfInterestBoxesLeft;

        /// Image pixel boxes (aka ftkRawData or region of interest) is the x, y location
        /// in the camera image of a suspected fiducial, plus the status
        /// and total count of pixels in the box, and the probability it
        /// is a fiducial.
        std::vector<ftkRawData> ImageRegionOfInterestBoxesRight;

        /// FTK_OK on success
        /// Indicates if one or more error occured. Note that with a fast frame grabbing loop
        /// FTK_WAR_NO_FRAME will be very common because no new data will be available,
        /// so be sure to account for this even if you only need bare-bones functionality.
        ///
        ///   \retval FTK_OK if the frame could be retrieved correctly,
        ///   \retval FTK_ERR_INIT if the \c frame pointer was not initialised using
        ///   ftkCreateFrame,
        ///   \retval FTK_ERR_INV_PTR if the \c lib handle was not correctly
        ///   initialised or if the \c FrameQueryPInOut is null or if the internal
        ///   data for the picture could not be allocated,
        ///   \retval FTK_ERR_INV_SN if the device could not be retrieved,
        ///   \retval FTK_ERR_INTERNAL if the triangulation or the marker matcher
        ///   class are not properly initialised  or if no image are retrieved or if
        ///   the image size is invalid or if a compressed image is corrupted,
        ///   \retval FTK_ERR_COMP_ALGO if the temperature compensation algorithm is
        ///   undefined,
        ///   \retval FTK_ERR_SYNC if the retrieved pictures are not synchronised,
        ///   \retval FTK_WAR_NO_FRAME if no frame are available,
        ///   \retval FTK_ERR_SEG_OVERFLOW if an overflow occurred during image segmentation,
        ///   \retval FTK_WAR_SHOCK_DETECTED A shock was detected such as from being dropped,
        ///           calibration may be lost, consider contacting Atracsys to service the device.
        ///   \retval FTK_WAR_TEMP_LOW The current temperature is too low for calibration compensation
        ///           to account for thermal contraction of the device.
        ///   \retval FTK_WAR_TEMP_HIGH The current temperature is too high for calibration compensation to account for thermal contraction of the device.
        ///
        /// @see ftkErrors.h
        /// @see ftkErrorExt
        ///
        /// @todo TODO(ahundt) remove the shared_ptr and make it a regular object once ftkErrorExt follows the C++ Rule of Three (https://en.wikipedia.org/wiki/Rule_of_three_(C%2B%2B_programming))
        ftkError Error;
        TimeEvent TimeStamp;
    };


    /// Create a Frame using the parameters defined when the FusionTrack class was created.
    ///
    /// @param serialNumber the serial number of the FusionTrack device you wish to receive data from,
    ///        the default of 0 will use the default device. You should only need to set this if more
    ///        than one FusionTrack device is connected.
    Frame makeFrame(uint64_t serialNumber = 0)
    {
        return Frame(
            serialNumber,
            m_params.retrieveLeftPixels,
            m_params.retrieveRightPixels,
            m_params.maxLeftImageRegionOfInterestBoxes,
            m_params.maxRightImageRegionOfInterestBoxes,
            m_params.max3DFiducialInstances,
            m_params.maxMarkerInstances
        );
    }

    /// Load Frame with data from the device specified in the
    /// Frame.SerialNumber field. If Frame.SerialNumber is 0 it
    /// Defaults to the first connected device. For data from
    /// another device specify the serial number.
    void receive(Frame& rs) {
        if(m_deviceSerialNumbers.size() == 0 ) throw std::runtime_error("FusionTrack::receive() called but no trackers are connected.");

        // default to the first device if none is specified
        if( rs.SerialNumber == 0 ) rs.SerialNumber = m_deviceSerialNumbers[0];

        // resize frame contents to the user specified capacity
        prepareFrameToReceiveData(rs);

        // get a local clock timestamp, then the latest frame from the device, then another timestamp
        rs.TimeStamp.local_request_time = cartographer::common::UniversalTimeScaleClock::now();
        ftkError error = ftkGetLastFrame(
            m_ftkLibrary,
            rs.SerialNumber,
            rs.FrameQueryP.get(),
            m_params.blockLimitMilliseconds /* block up to limit (ex: 100 ms) if next frame is not available*/);
        rs.TimeStamp.local_receive_time = cartographer::common::UniversalTimeScaleClock::now();

        ftkErrorExt errors(rs.Error);

        rs.Error = error;
        if (error != FTK_OK) {
            // provide clean and useful errors
            ftkGetLastError( m_ftkLibrary, &errors);

            #ifdef HAVE_SPDLOG
            if(m_logger && errors.isWarning() && errors.isError()) {
                std::string error;
                errors.messageStack(error);
                m_logger->error(std::string("FusionTrack::receive() encountered both warnings and errors:\n") + error);
            } else if(m_logger && errors.isWarning()) {
                std::string warning;
                errors.warningString(warning);
                m_logger->warn(std::string("FusionTrack::receive():") + warning);

            } else if(m_logger && errors.isError()) {
                std::string error;
                errors.errorString(error);
                m_logger->error(std::string("FusionTrack::receive():") + error);
            }
            #endif // HAVE_SPDLOG

            if(errors.isError())
            {
                // don't do any additional processing upon errors
                return;
            }
        }

        // When you get here, either everything is ok or there is only a warning.
        // Adjust the size of Frame contents according to the actual amount of data received.
        correctFrameAfterReceivingData(rs);

        // check results of last frame
        switch (rs.FrameQueryP->markersStat) {
            case QS_WAR_SKIPPED:
                #ifdef HAVE_SPDLOG
                if(m_logger) m_logger->error("FusionTrack::receive: marker fields in the frame are not set correctly");
                #endif // HAVE_SPDLOG
            case QS_ERR_INVALID_RESERVED_SIZE:
                #ifdef HAVE_SPDLOG
                if(m_logger) m_logger->error("FusionTrack::receive: FrameQueryP->markersVersionSize is invalid");
                #endif // HAVE_SPDLOG
            case QS_OK:
                break;
            default:
                #ifdef HAVE_SPDLOG
                if(m_logger) m_logger->error("FusionTrack::receive: invalid status of value: ", rs.FrameQueryP->markersStat);
                #endif // HAVE_SPDLOG
                break;
        }

        // convert the device time to a common timestamp
        rs.TimeStamp.device_time = ImageHeaderToCommonTime(*(rs.FrameQueryP->imageHeader));

        // make sure we're not getting more markers than allocated
        auto count = rs.FrameQueryP->markersCount;
        if (count > rs.Markers.capacity()) {
            #ifdef HAVE_SPDLOG
            if(m_logger) m_logger->warn("FusionTrack::receive: marker overflow, please increase number of markers.  Only the first ", rs.Markers.size(), " marker(s) will processed.");
            #endif // HAVE_SPDLOG
        }
    }

    /// Get the serial numbers of all connected devices.
    std::vector<uint64_t> getDeviceSerialNumbers() const {
        return m_deviceSerialNumbers;
    }

    /// Internal function to register connected devices
    /// Not for users to utilize. TODO(ahundt) make private
    void internalAddDevice(uint64_t id, uint8_t type)
    {
        m_deviceSerialNumbers.push_back(id);
        m_device_types.push_back(type);
    }

    int8_t getTypeFromSerialNumber(uint64_t id) {
        ptrdiff_t pos = std::distance(m_deviceSerialNumbers.begin(), std::find(m_deviceSerialNumbers.begin(), m_deviceSerialNumbers.end(), id));
        uint8_t type = *(m_device_types.begin() + pos);
        return type;
    }
    std::vector<uint8_t> getDeviceTypes() const {
        return  m_device_types;
    }
    
    ~FusionTrack(){
       ftkClose(&m_ftkLibrary);
    }

    Params getParams() const {
        return m_params;
    }
private:

    /// Prepare the Frame contents for a new data update,
    /// accounting for the maximum amount of data that might arrive.
    void prepareFrameToReceiveData(FusionTrack::Frame& frame) {
        frame.Fiducials.resize(frame.FrameQueryP->threeDFiducialsVersionSize.ReservedSize/sizeof(ftk3DFiducial), ftk3DFiducial());
        frame.Markers.resize(frame.FrameQueryP->markersVersionSize.ReservedSize/sizeof(ftkMarker), ftkMarker());
        frame.ImageRegionOfInterestBoxesLeft.resize(frame.FrameQueryP->rawDataLeftVersionSize.ReservedSize/sizeof(ftkRawData), ftkRawData());
        frame.ImageRegionOfInterestBoxesRight.resize(frame.FrameQueryP->rawDataRightVersionSize.ReservedSize/sizeof(ftkRawData), ftkRawData());
    }

    /// Correct the Frame contents to reflect actual data received.
    void correctFrameAfterReceivingData(FusionTrack::Frame& frame) {
        frame.Fiducials.resize(frame.FrameQueryP->threeDFiducialsCount, ftk3DFiducial());
        frame.Markers.resize(frame.FrameQueryP->markersCount, ftkMarker());
        frame.ImageRegionOfInterestBoxesLeft.resize(frame.FrameQueryP->rawDataLeftCount, ftkRawData());
        frame.ImageRegionOfInterestBoxesRight.resize(frame.FrameQueryP->rawDataRightCount, ftkRawData());
        frame.DeviceType = ftkDeviceType(getTypeFromSerialNumber(frame.SerialNumber));
    }

    #ifdef HAVE_SPDLOG
    std::shared_ptr<spdlog::logger> m_logger;
    #endif // HAVE_SPDLOG
    Params m_params;
    ftkLibrary m_ftkLibrary;
    // device serial numbers
    std::vector<uint64_t> m_deviceSerialNumbers;
    // fusiontrack vs infinitrack
    std::vector<uint8_t> m_device_types;

};


namespace detail {
    void updateDeviceSerialNumber(uint64 device, void * user, ftkDeviceType type)
    {
        FusionTrack * driver = reinterpret_cast<FusionTrack *>(user);
        driver->internalAddDevice(device,type);
    }
}

}} // grl::sensor


#endif

