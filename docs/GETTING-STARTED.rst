Getting Started with NLSR
=========================

.. toctree::
..

Getting Source
--------------

-  `GitHub NLSR repository <https://github.com/named-data/NLSR>`_

Installation
------------

:doc:`INSTALL`


Important Notes About Configuration
-----------------------------------

Since v0.4.0, NLSR no longer creates Faces for the neighbors that are
specified in ``nlsr.conf``. Instead, it relies on the pre-existence of
the Faces in NFD. NLSR will obtain this information from NFD, and
configure its neighbors using this information.

For people who were relying on NLSR to automagically configure their
networks, this must now be done with ``nfdc``. Its documentation can
be found `here
<https://named-data.net/doc/NFD/current/manpages/nfdc.html>`_

Configuration
-------------

After installing NLSR from source, you need to create a configuration file for
NLSR. Please take a look at :doc:`manpages/nlsr.conf` for a sample configuration. For
details on configuring a router, please refer to :doc:`ROUTER-CONFIG`. For details on
security configuration, please refer to :doc:`SECURITY-CONFIG`.

Running
-------

Run nlsr with the following command:

::

    nlsr

NLSR will look for the default configuration file, ``nlsr.conf``, in the current directory.

You can also run ``nlsr -f`` with the absolute path of the configuration file:

::

    nlsr -f /usr/local/etc/ndn/nlsr.conf

Logging
-------

NLSR uses the ndn-cxx logging facility. All levels listed below the selected log-level
value are enabled.

::

    Valid values:

      TRACE    trace messages (most verbose)
      DEBUG    debugging messages
      INFO     informational messages
      WARN     warning messages
      ERROR    error messages
      FATAL    fatal (will be logged unconditionally)

To obtain logs for NLSR, set the NDN_LOG environment variable with the correct prefix and
log-level settings. For example, running the following command will display all log
messages in NLSR with a DEBUG level or below.

::

    NDN_LOG='nlsr.*=DEBUG' nlsr

If the user is presented with an error message `User does not have read and write
permission on the directory` it can be circumvented by running the application with sudo:

::

    sudo env NDN_LOG='nlsr.*=DEBUG' nlsr

Use `man ndn-log` for more detailed instructions.
