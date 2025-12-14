ONNX Runtime
============

Introduction
------------

ONNX Runtime executes ONNX models. It is a library that runs ONNX models on
different platforms and devices.

ONNX Runtime Tensor Handling
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

In C++ with ONNX Runtime, tensors are handled as wrapper objects around
contiguous memory arrays with shape metadata. The implementation follows these
steps:

1. Tensor creation from C++ data: The formatted observation vector
   (``std::vector<float>``) is wrapped into an ``Ort::Value`` tensor using
   ``Ort::Value::CreateTensor<float>()``. This function takes:
   - Memory allocator information
   - Pointer to the underlying C++ array data (from ``std::vector::data()``)
   - Total number of elements
   - Shape array (``std::vector<int64_t>``) defining tensor dimensions
   - Number of dimensions

   The tensor does not copy the data; it references the original memory,
   avoiding unnecessary memory allocation.

2. Tensor shape: The shape is represented as ``std::vector<int64_t>``, for
   example ``[1, 45]`` for a batch size of 1 and 45 features. Dynamic
   dimensions (marked as -1 in the model) are resolved at runtime based on
   the actual input size.

3. Tensor type: The element type is specified by ``ONNXTensorElementDataType``
   enum (e.g., ``ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT`` for float32). Most ML
   models use float32 precision, which matches the C++ ``float`` type.

4. Inference execution: The input tensor is passed to ``onnx_session_->Run()``
   along with input/output name pointers. ONNX Runtime performs the inference
   and returns output tensors.

5. Tensor extraction: After inference, output tensors are accessed via:

   - ``GetTensorMutableData<float>()``: Returns a typed pointer (``float*``)
     to the underlying tensor data

   - ``GetTensorTypeAndShapeInfo().GetElementCount()``: Returns the total
     number of elements

   The output data is then copied into a ``std::vector<double>`` for
   compatibility with ROS2 control interfaces, which use double precision.

This approach avoids data copying during tensor creation while maintaining
type safety and compatibility with both ONNX Runtime (float32) and ROS2
control interfaces (float64).

Model Metadata Inspection
~~~~~~~~~~~~~~~~~~~~~~~~~~

The controller includes utility functions for logging and validating model
metadata during initialization. The ``log_input_metadata()`` function
inspects the loaded ONNX model structure:

- ``Ort::Session & session``: Reference to the ONNX Runtime session
  containing the loaded model. Used to query input metadata such as input
  count, names, types, and shapes.

- ``Ort::AllocatorWithDefaultOptions & allocator``: Reference to an ONNX
  Runtime memory allocator. Required when retrieving string names from the
  session (e.g., ``GetInputNameAllocated()`` needs an allocator to return
  the name string).

Both arguments are passed by reference to avoid copying and enable the
function to query the session and use the allocator for string operations.

Outputs:

- Model inference returns relative joint positions (N joints, scaled by 0.25 per
  ``env_cfg.py``)
- ActionProcessor processes model outputs:

  - Applies scale factor (0.25) to model outputs
  - Adds default joint position offset (initialized from sensor data on first
    update)
  - Converts relative positions to absolute joint positions

- Write absolute joint positions to hardware position command interfaces
