-- license:BSD-3-Clause
-- copyright-holders:MAMEdev Team

---------------------------------------------------------------------------
--
--   scv.lua
--
--   SCV-specific makefile
--   Use make SUBTARGET=scv to build
--
---------------------------------------------------------------------------


--------------------------------------------------
-- Specify all the CPU cores necessary for the
-- drivers referenced in scv.lst.
--------------------------------------------------

CPUS["UPD7810"] = true
CPUS["UPD177X"] = true

--------------------------------------------------
-- Specify all the sound cores necessary for the
-- drivers referenced in scv.lst.
--------------------------------------------------

SOUNDS["UPD1771C_017"] = true


--------------------------------------------------
-- specify available machine cores
--------------------------------------------------

MACHINES["SCV"] = true


--------------------------------------------------
-- specify available bus cores
--------------------------------------------------

BUSES["SCV"] = true


--------------------------------------------------
-- This is the list of files that are necessary
-- for building all of the drivers referenced
-- in scv.lst
--------------------------------------------------

function createProjects_mame_scv(_target, _subtarget)
	project ("mame_scv")
	targetsubdir(_target .."_" .. _subtarget)
	kind (LIBTYPE)
	uuid (os.uuid("drv-mame-scv"))
	addprojectflags()
	precompiledheaders_novs()

	includedirs {
		MAME_DIR .. "src/osd",
		MAME_DIR .. "src/emu",
		MAME_DIR .. "src/devices",
		MAME_DIR .. "src/mame/shared",
		MAME_DIR .. "src/lib",
		MAME_DIR .. "src/lib/util",
		MAME_DIR .. "3rdparty",
		GEN_DIR  .. "mame/layout",
	}

files{
	MAME_DIR .. "src/mame/epoch/scv.cpp",
}
end

function linkProjects_mame_scv(_target, _subtarget)
	links {
		"mame_scv",
	}
end
