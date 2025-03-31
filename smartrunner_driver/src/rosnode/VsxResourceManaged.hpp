/* SPDX-License-Identifier: Apache-2.0 */

/* Copyright 2025 Pepperl+Fuchs SE
 *
 * Authors:
 *   Adrian Krzizok <git@breeze-innovations.com>
 *   Jan Hegner <opensource-jhe@de.pepperl-fuchs.com>
 *   Markus Moll <opensource-mmo@de.pepperl-fuchs.com>
 */

#ifndef ROSNODE__VSXRESOURCEMANAGED_HPP_
#define ROSNODE__VSXRESOURCEMANAGED_HPP_

#include <utility>

#include <PF.VsxProtocolDriver.WrapperNE.h>

namespace pepperl_fuchs
{

template<typename T, VsxStatusCode Deleter(T **)>
class VsxResourceManaged
{
public:
  VsxResourceManaged() = default;

  ~VsxResourceManaged() {Free();}

  /// <summary>
  /// Construct managed class with an existing pointer to a resource
  /// Note: The ownership is transfered to this object.
  /// </summary>
  /// <param name="resource"></param>
  explicit VsxResourceManaged(T * resource)
  : mValue(resource) {}

  /// <summary>
  /// deleted copy-constructor
  /// </summary>
  VsxResourceManaged(const VsxResourceManaged &) = delete;

  /// <summary>
  /// deleted assignment operator
  /// </summary>
  VsxResourceManaged & operator=(const VsxResourceManaged &) = delete;

  /// <summary>
  /// Move constructor
  /// Takes over ownership of object that had the ownership before
  /// </summary>
  VsxResourceManaged(VsxResourceManaged && that) noexcept
  : mValue(that.mValue) {that.mValue = nullptr;}

  /// <summary>
  /// Move assignment operator
  /// This object takes over ownership from previous object.
  /// </summary>
  /// <param name="that"></param>
  /// <returns></returns>
  VsxResourceManaged & operator=(VsxResourceManaged && that) noexcept
  {
    Reset(that.mValue);
    that.mValue = nullptr;
    return *this;
  }

  /// <summary>
  /// Get pointer to underlying resource that can be used for initialization with
  /// vsx_*-functions.
  /// If the object already holds a resource. The resource is freed first
  /// </summary>
  /// <returns>Pointer to underlying resource location for init</returns>
  T ** GetPointerForInit()
  {
    if (mValue) {Free();}
    return &mValue;
  }

  /// <summary>
  /// Check if this object is holding a resource
  /// </summary>
  explicit operator bool() const {return mValue != nullptr;}

  /// <summary>
  /// Return a pointer to the underlying resource. This allows for easy
  /// passing of an existing resource as parameter to vsx_*-functions.
  /// Example:
  /// result = vsx_GetImage(dataContainer, "Intensity", image.GetPointerForInit())
  /// dataContainer-parameter uses operator T*() for usage
  /// image-parameter uses GetPointerForInit() for initialization
  /// </summary>
  operator T *() const {return mValue;}

  /// <summary>
  /// operator-> for access of struct members
  /// </summary>
  T * operator->() const {return mValue;}

  /// <summary>
  /// * operator for dereferencing
  /// </summary>
  /// <returns>Reference to resource</returns>
  T & operator*() const {return *mValue;}

  /// <summary>
  /// Get pointer to underlying resource
  /// </summary>
  T * Get() const {return mValue;}

  /// <summary>
  /// Release ownership of managed resource
  /// The resource is thus not managed by this object any more
  /// and will not be destroyed when this is destroyed
  /// </summary>
  /// <returns>Pointer to resource</returns>
  T * Release()
  {
    T * temp = mValue;
    mValue = nullptr;
    return temp;
  }

  /// <summary>
  /// Take ownership of a new resource
  /// Existing resources will be destroyed first
  /// </summary>
  /// <param name="newValue">Pointer to ne resource</param>
  void Reset(T * newValue)
  {
    Free();
    mValue = newValue;
  }

  /// <summary>
  /// Swap to VsxResourceManaged-objects
  /// </summary>
  /// <param name="other">other managed object to swap</param>
  void Swap(VsxResourceManaged & other) {std::swap(mValue, other.mValue);}

private:
  void Free() {Deleter(&mValue);}

  T * mValue = nullptr;
};

using VsxDeviceListManaged = VsxResourceManaged<VsxDeviceList, vsx_ReleaseDeviceList>;
using VsxImageManaged = VsxResourceManaged<VsxImage, vsx_ReleaseImage>;
using VsxDisparityDescriptor2Managed = VsxResourceManaged<VsxDisparityDescriptor2,
    vsx_ReleaseDisparityDescriptor2>;
using VsxTransformationManaged = VsxResourceManaged<VsxTransformation, vsx_ReleaseTransformation>;
using VsxCaptureInformationManaged = VsxResourceManaged<VsxCaptureInformation,
    vsx_ReleaseCaptureInformation>;
using VsxDataContainerManaged = VsxResourceManaged<VsxDataContainerHandle,
    vsx_ReleaseDataContainer>;
using VsxSystemHandleManaged = VsxResourceManaged<VsxSystemHandle, vsx_ReleaseSensor>;
using VsxStringManaged = VsxResourceManaged<const char, vsx_ReleaseString>;
using VsxDeviceManaged = VsxResourceManaged<VsxDevice, vsx_ReleaseDevice>;
using VsxLineDataManaged = VsxResourceManaged<VsxLineData, vsx_ReleaseLine>;
using VsxOlr2CaptureInformationManaged =
  VsxResourceManaged<VsxOlr2CaptureInformation, vsx_ReleaseOlr2CaptureInformation>;
using VsxOlr2ModbusDataManaged = VsxResourceManaged<VsxOlr2ModbusData, vsx_ReleaseOlr2ModbusData>;
using VsxTagListManaged = VsxResourceManaged<VsxTagList, vsx_ReleaseTagList>;
using VsxParameterManaged = VsxResourceManaged<const VsxParameter, vsx_ReleaseParameter>;
using VsxParameterListManaged = VsxResourceManaged<VsxParameterList, vsx_ReleaseParameterList>;
using VsxStatusItemListManaged = VsxResourceManaged<VsxStatusItemList, vsx_ReleaseStatusItemList>;

}  // namespace pepperl_fuchs

#endif  // ROSNODE__VSXRESOURCEMANAGED_HPP_
