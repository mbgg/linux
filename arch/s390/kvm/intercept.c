/*
 * in-kernel handling for sie intercepts
 *
 * Copyright IBM Corp. 2008, 2009
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License (version 2 only)
 * as published by the Free Software Foundation.
 *
 *    Author(s): Carsten Otte <cotte@de.ibm.com>
 *               Christian Borntraeger <borntraeger@de.ibm.com>
 */

#include <linux/kvm_host.h>
#include <linux/errno.h>
#include <linux/pagemap.h>

#include <asm/kvm_host.h>

#include "kvm-s390.h"
#include "gaccess.h"
#include "trace.h"
#include "trace-s390.h"

static int handle_lctlg(struct kvm_vcpu *vcpu)
{
	int reg1 = (vcpu->arch.sie_block->ipa & 0x00f0) >> 4;
	int reg3 = vcpu->arch.sie_block->ipa & 0x000f;
	u64 useraddr;
	int reg, rc;

	vcpu->stat.instruction_lctlg++;

	useraddr = kvm_s390_get_base_disp_rsy(vcpu);

	if (useraddr & 7)
		return kvm_s390_inject_program_int(vcpu, PGM_SPECIFICATION);

	reg = reg1;

	VCPU_EVENT(vcpu, 5, "lctlg r1:%x, r3:%x, addr:%llx", reg1, reg3,
		   useraddr);
	trace_kvm_s390_handle_lctl(vcpu, 1, reg1, reg3, useraddr);

	do {
		rc = get_guest(vcpu, vcpu->arch.sie_block->gcr[reg],
			       (u64 __user *) useraddr);
		if (rc) {
			kvm_s390_inject_program_int(vcpu, PGM_ADDRESSING);
			break;
		}
		useraddr += 8;
		if (reg == reg3)
			break;
		reg = (reg + 1) % 16;
	} while (1);
	return 0;
}

static int handle_lctl(struct kvm_vcpu *vcpu)
{
	int reg1 = (vcpu->arch.sie_block->ipa & 0x00f0) >> 4;
	int reg3 = vcpu->arch.sie_block->ipa & 0x000f;
	u64 useraddr;
	u32 val = 0;
	int reg, rc;

	vcpu->stat.instruction_lctl++;

	useraddr = kvm_s390_get_base_disp_rs(vcpu);

	if (useraddr & 3)
		return kvm_s390_inject_program_int(vcpu, PGM_SPECIFICATION);

	VCPU_EVENT(vcpu, 5, "lctl r1:%x, r3:%x, addr:%llx", reg1, reg3,
		   useraddr);
	trace_kvm_s390_handle_lctl(vcpu, 0, reg1, reg3, useraddr);

	reg = reg1;
	do {
		rc = get_guest(vcpu, val, (u32 __user *) useraddr);
		if (rc) {
			kvm_s390_inject_program_int(vcpu, PGM_ADDRESSING);
			break;
		}
		vcpu->arch.sie_block->gcr[reg] &= 0xffffffff00000000ul;
		vcpu->arch.sie_block->gcr[reg] |= val;
		useraddr += 4;
		if (reg == reg3)
			break;
		reg = (reg + 1) % 16;
	} while (1);
	return 0;
}

static const intercept_handler_t eb_handlers[256] = {
	[0x2f] = handle_lctlg,
	[0x8a] = kvm_s390_handle_priv_eb,
};

static int handle_eb(struct kvm_vcpu *vcpu)
{
	intercept_handler_t handler;

	handler = eb_handlers[vcpu->arch.sie_block->ipb & 0xff];
	if (handler)
		return handler(vcpu);
	return -EOPNOTSUPP;
}

static const intercept_handler_t instruction_handlers[256] = {
	[0x01] = kvm_s390_handle_01,
	[0x82] = kvm_s390_handle_lpsw,
	[0x83] = kvm_s390_handle_diag,
	[0xae] = kvm_s390_handle_sigp,
	[0xb2] = kvm_s390_handle_b2,
	[0xb7] = handle_lctl,
	[0xb9] = kvm_s390_handle_b9,
	[0xe5] = kvm_s390_handle_e5,
	[0xeb] = handle_eb,
};

static int handle_noop(struct kvm_vcpu *vcpu)
{
	switch (vcpu->arch.sie_block->icptcode) {
	case 0x0:
		vcpu->stat.exit_null++;
		break;
	case 0x10:
		vcpu->stat.exit_external_request++;
		break;
	case 0x14:
		vcpu->stat.exit_external_interrupt++;
		break;
	default:
		break; /* nothing */
	}
	return 0;
}

static int handle_stop(struct kvm_vcpu *vcpu)
{
	int rc = 0;

	vcpu->stat.exit_stop_request++;
	spin_lock_bh(&vcpu->arch.local_int.lock);

	trace_kvm_s390_stop_request(vcpu->arch.local_int.action_bits);

	if (vcpu->arch.local_int.action_bits & ACTION_RELOADVCPU_ON_STOP) {
		vcpu->arch.local_int.action_bits &= ~ACTION_RELOADVCPU_ON_STOP;
		rc = SIE_INTERCEPT_RERUNVCPU;
		vcpu->run->exit_reason = KVM_EXIT_INTR;
	}

	if (vcpu->arch.local_int.action_bits & ACTION_STOP_ON_STOP) {
		atomic_set_mask(CPUSTAT_STOPPED,
				&vcpu->arch.sie_block->cpuflags);
		vcpu->arch.local_int.action_bits &= ~ACTION_STOP_ON_STOP;
		VCPU_EVENT(vcpu, 3, "%s", "cpu stopped");
		rc = -EOPNOTSUPP;
	}

	if (vcpu->arch.local_int.action_bits & ACTION_STORE_ON_STOP) {
		vcpu->arch.local_int.action_bits &= ~ACTION_STORE_ON_STOP;
		/* store status must be called unlocked. Since local_int.lock
		 * only protects local_int.* and not guest memory we can give
		 * up the lock here */
		spin_unlock_bh(&vcpu->arch.local_int.lock);
		rc = kvm_s390_vcpu_store_status(vcpu,
						KVM_S390_STORE_STATUS_NOADDR);
		if (rc >= 0)
			rc = -EOPNOTSUPP;
	} else
		spin_unlock_bh(&vcpu->arch.local_int.lock);
	return rc;
}

static int handle_validity(struct kvm_vcpu *vcpu)
{
	unsigned long vmaddr;
	int viwhy = vcpu->arch.sie_block->ipb >> 16;
	int rc;

	vcpu->stat.exit_validity++;
	trace_kvm_s390_intercept_validity(vcpu, viwhy);
	if (viwhy == 0x37) {
		vmaddr = gmap_fault(vcpu->arch.sie_block->prefix,
				    vcpu->arch.gmap);
		if (IS_ERR_VALUE(vmaddr)) {
			rc = -EOPNOTSUPP;
			goto out;
		}
		rc = fault_in_pages_writeable((char __user *) vmaddr,
			 PAGE_SIZE);
		if (rc) {
			/* user will receive sigsegv, exit to user */
			rc = -EOPNOTSUPP;
			goto out;
		}
		vmaddr = gmap_fault(vcpu->arch.sie_block->prefix + PAGE_SIZE,
				    vcpu->arch.gmap);
		if (IS_ERR_VALUE(vmaddr)) {
			rc = -EOPNOTSUPP;
			goto out;
		}
		rc = fault_in_pages_writeable((char __user *) vmaddr,
			 PAGE_SIZE);
		if (rc) {
			/* user will receive sigsegv, exit to user */
			rc = -EOPNOTSUPP;
			goto out;
		}
	} else
		rc = -EOPNOTSUPP;

out:
	if (rc)
		VCPU_EVENT(vcpu, 2, "unhandled validity intercept code %d",
			   viwhy);
	return rc;
}

static int handle_instruction(struct kvm_vcpu *vcpu)
{
	intercept_handler_t handler;

	vcpu->stat.exit_instruction++;
	trace_kvm_s390_intercept_instruction(vcpu,
					     vcpu->arch.sie_block->ipa,
					     vcpu->arch.sie_block->ipb);
	handler = instruction_handlers[vcpu->arch.sie_block->ipa >> 8];
	if (handler)
		return handler(vcpu);
	return -EOPNOTSUPP;
}

static int handle_prog(struct kvm_vcpu *vcpu)
{
	vcpu->stat.exit_program_interruption++;
	trace_kvm_s390_intercept_prog(vcpu, vcpu->arch.sie_block->iprcc);
	return kvm_s390_inject_program_int(vcpu, vcpu->arch.sie_block->iprcc);
}

static int handle_instruction_and_prog(struct kvm_vcpu *vcpu)
{
	int rc, rc2;

	vcpu->stat.exit_instr_and_program++;
	rc = handle_instruction(vcpu);
	rc2 = handle_prog(vcpu);

	if (rc == -EOPNOTSUPP)
		vcpu->arch.sie_block->icptcode = 0x04;
	if (rc)
		return rc;
	return rc2;
}

static const intercept_handler_t intercept_funcs[] = {
	[0x00 >> 2] = handle_noop,
	[0x04 >> 2] = handle_instruction,
	[0x08 >> 2] = handle_prog,
	[0x0C >> 2] = handle_instruction_and_prog,
	[0x10 >> 2] = handle_noop,
	[0x14 >> 2] = handle_noop,
	[0x18 >> 2] = handle_noop,
	[0x1C >> 2] = kvm_s390_handle_wait,
	[0x20 >> 2] = handle_validity,
	[0x28 >> 2] = handle_stop,
};

int kvm_handle_sie_intercept(struct kvm_vcpu *vcpu)
{
	intercept_handler_t func;
	u8 code = vcpu->arch.sie_block->icptcode;

	if (code & 3 || (code >> 2) >= ARRAY_SIZE(intercept_funcs))
		return -EOPNOTSUPP;
	func = intercept_funcs[code >> 2];
	if (func)
		return func(vcpu);
	return -EOPNOTSUPP;
}
