:mod:`motor` -- Motor class
===========================

.. automodule:: pypot.dynamixel.motor

The motor classes all inherit from the :class:`DynamixelMotor` class. The end-user class use multiple inheritance to adapt to the model specificities.

.. autoclass:: AXMotor
   :show-inheritance:
   :members:
   :member-order: by source

.. autoclass:: RXMotor
   :show-inheritance:
   :members:
   :member-order: by source

.. autoclass:: MXMotor
   :show-inheritance:
   :members:
   :member-order: by source

.. autoclass:: VXMotor
   :show-inheritance:
   :members:
   :member-order: by source


.. autoclass:: DynamixelMotor
   :members:
   :undoc-members:
   :member-order: bysource

   .. automethod:: __init__

This extra properties only apply to specific class of models.

.. autoclass:: ComplianceMarginSlopeExtra
   :members:
   :undoc-members:
   :member-order: by source

.. autoclass:: PIDExtra
   :members:
   :undoc-members:
   :member-order: by source

.. autoclass:: GoalAccelerationExtra
   :members:
   :undoc-members:
   :member-order: by source

.. autoclass:: CurrentExtra
   :members:
   :undoc-members:
   :member-order: by source

.. autoclass:: SensedCurrentExtra
   :members:
   :undoc-members:
   :member-order: by source

